/*
 *
 *    Copyright (c) 2020-2021 Project CHIP Authors
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

/**
 *    @file
 *          Provides an implementation of the BLEManager singleton object
 *          for mbed platforms.
 */

#include <platform/internal/CHIPDeviceLayerInternal.h>

#if CHIP_DEVICE_CONFIG_ENABLE_CHIPOBLE

#include <platform/mbed/BLEManagerImpl.h>

#include <ble/CHIPBleServiceData.h>
#include <platform/internal/BLEManager.h>
#include <support/CodeUtils.h>
#include <support/logging/CHIPLogging.h>

// FIXME temporary macros
#define _BLEMGRIMPL_USE_LEDS 1
#define _BLEMGRIMPL_USE_MBED_EVENTS 1
// Disable advertising autostart for development.
#define CHIP_DEVICE_CONFIG_CHIPOBLE_ENABLE_ADVERTISING_AUTOSTART 0

/* Undefine the BLE_ERROR_NOT_IMPLEMENTED macro provided by CHIP's
 * src/ble/BleError.h to avoid a name conflict with Mbed-OS ble_error_t
 * enum value. For the enum values, see:
 * mbed-os/connectivity/FEATURE_BLE/include/ble/common/blecommon.h
 */
#undef BLE_ERROR_NOT_IMPLEMENTED
// mbed-os headers
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "platform/Callback.h"

#if _BLEMGRIMPL_USE_LEDS
#include "drivers/DigitalOut.h"
#endif

#if _BLEMGRIMPL_USE_MBED_EVENTS
#include "events/mbed_events.h"
#include "rtos/Thread.h"
#endif

using namespace ::chip;
using namespace ::chip::Ble;
using namespace ::chip::System;

namespace chip {
namespace DeviceLayer {
namespace Internal {

namespace {
const ChipBleUUID ChipUUID_CHIPoBLEChar_RX = { { 0x18, 0xEE, 0x2E, 0xF5, 0x26, 0x3D, 0x45, 0x59, 0x95, 0x9F, 0x4F, 0x9C, 0x42, 0x9F,
                                                 0x9D, 0x11 } };
const ChipBleUUID ChipUUID_CHIPoBLEChar_TX = { { 0x18, 0xEE, 0x2E, 0xF5, 0x26, 0x3D, 0x45, 0x59, 0x95, 0x9F, 0x4F, 0x9C, 0x42, 0x9F,
                                                 0x9D, 0x12 } };
} // namespace

#if _BLEMGRIMPL_USE_LEDS
#include "drivers/DigitalOut.h"
// LED1 -- toggle on every call to ble::BLE::processEvents()
// LED2 -- on when ble::BLE::init() callback completes
// LED3 -- on when advertising
mbed::DigitalOut led1(LED1, 1);
mbed::DigitalOut led2(LED2, 1);
mbed::DigitalOut led3(LED3, 1);
#endif

BLEManagerImpl BLEManagerImpl::sInstance;

// FIXME use CHIP platform for deferred calls
#if _BLEMGRIMPL_USE_MBED_EVENTS
events::EventQueue event_queue(32 * EVENTS_EVENT_SIZE);
rtos::Thread event_thread;
#endif

CHIP_ERROR BLEManagerImpl::_Init()
{
    CHIP_ERROR err       = CHIP_NO_ERROR;
    ble_error_t mbed_err = BLE_ERROR_NONE;

#if _BLEMGRIMPL_USE_MBED_EVENTS
    event_thread.start(mbed::callback(&event_queue, &events::EventQueue::dispatch_forever));
    ChipLogDetail(DeviceLayer, "Processing BLE events with Mbed EventQueue");
#else
    ChipLogDetail(DeviceLayer, "Processing BLE events with CHIP PlatformMgr");
#endif

    mServiceMode = ConnectivityManager::kCHIPoBLEServiceMode_Enabled;
    mFlags       = CHIP_DEVICE_CONFIG_CHIPOBLE_ENABLE_ADVERTISING_AUTOSTART ? kFlag_AdvertisingEnabled : 0;
    mGAPConns    = 0;
    // memset(mSubscribedConns, 0, sizeof(mSubscribedConns));

    ble::BLE & ble_interface = ble::BLE::Instance();

    ble_interface.onEventsToProcess(FunctionPointerWithContext<ble::BLE::OnEventsToProcessCallbackContext *>{
        [](ble::BLE::OnEventsToProcessCallbackContext * context) {
#if _BLEMGRIMPL_USE_MBED_EVENTS
            event_queue.call(DoBLEProcessing, 0);
#else
            PlatformMgr().ScheduleWork(DoBLEProcessing, 0);
#endif
        } });

    mbed_err = ble_interface.init([](ble::BLE::InitializationCompleteCallbackContext * context) {
        BLEMgrImpl().HandleInitComplete(context->error == BLE_ERROR_NONE);
    });
    VerifyOrExit(mbed_err == BLE_ERROR_NONE, err = CHIP_ERROR_INTERNAL);

exit:
    return err;
}

void BLEManagerImpl::DoBLEProcessing(intptr_t arg)
{
#if _BLEMGRIMPL_USE_LEDS
    led1 = !led1;
#endif
    // FIXME remove logging
    static int n = 0;
    ChipLogDetail(DeviceLayer, "ble-%02d", n++);
    ble::BLE::Instance().processEvents();
}

void BLEManagerImpl::HandleInitComplete(bool no_error)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(no_error, err = CHIP_ERROR_INTERNAL);

#if _IMPL_READY_BLEPLATFORMDELEGATE && _IMPL_READY_BLEAPPLICATIONDELEGATE
    err = BleLayer::Init(this, this, &SystemLayer);
    SuccessOrExit(err);
#endif
    PlatformMgr().ScheduleWork(DriveBLEState, 0);
#if _BLEMGRIMPL_USE_LEDS
    led2 = 0;
#endif

exit:
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "BLEManager init error: %s ", ErrorStr(err));
        ChipLogError(DeviceLayer, "Disabling CHIPoBLE service.");
        mServiceMode = ConnectivityManager::kCHIPoBLEServiceMode_Disabled;
        PlatformMgr().ScheduleWork(DriveBLEState, 0);
    }
}

void BLEManagerImpl::DriveBLEState(intptr_t arg)
{
    BLEMgrImpl().DriveBLEState();
}

// TODO verify the logic here
void BLEManagerImpl::DriveBLEState()
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    // Perform any initialization actions that must occur after the CHIP task is running.
    if (!GetFlag(mFlags, kFlag_AsyncInitCompleted))
    {
        SetFlag(mFlags, kFlag_AsyncInitCompleted);

        // If CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED is enabled,
        // disable CHIPoBLE advertising if the device is fully provisioned.
#if CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED
        if (ConfigurationMgr().IsFullyProvisioned())
        {
            ClearFlag(mFlags, kFlag_AdvertisingEnabled);
            ChipLogProgress(DeviceLayer, "CHIPoBLE advertising disabled because device is fully provisioned");
        }
#endif // CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED
    }

    // If the application has enabled CHIPoBLE and BLE advertising...
    if (mServiceMode == ConnectivityManager::kCHIPoBLEServiceMode_Enabled &&
        GetFlag(mFlags, kFlag_AdvertisingEnabled)
#if CHIP_DEVICE_CONFIG_CHIPOBLE_SINGLE_CONNECTION
        // and no connections are active...
        && (_NumConnections() == 0)
#endif
    )
    {
        // Start/re-start advertising if not already advertising, or if the
        // advertising state needs to be refreshed.
        if (!GetFlag(mFlags, kFlag_Advertising) || GetFlag(mFlags, kFlag_AdvertisingRefreshNeeded))
        {
            err = StartAdvertising();
            SuccessOrExit(err);
        }
    }
    // Otherwise, stop advertising if currently active.
    else
    {
        err = StopAdvertising();
        SuccessOrExit(err);
    }

exit:
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "Disabling CHIPoBLE service due to error: %s", ErrorStr(err));
        mServiceMode = ConnectivityManager::kCHIPoBLEServiceMode_Disabled;
    }
}

CHIP_ERROR BLEManagerImpl::StartAdvertising(void)
{
    CHIP_ERROR err       = CHIP_NO_ERROR;
    ble_error_t mbed_err = BLE_ERROR_NONE;

    ble::BLE & ble_interface = ble::BLE::Instance();
    ble::AdvertisingDataBuilder adv_data_builder(mAdvertisingDataBuffer);

    ble::AdvertisingParameters adv_params(ble::advertising_type_t::SCANNABLE_UNDIRECTED,
                                          ble::adv_interval_t(ble::millisecond_t(1000)));
    mbed_err = ble_interface.gap().setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, adv_params);
    VerifyOrExit(mbed_err == BLE_ERROR_NONE, err = CHIP_ERROR_INTERNAL);

    if (!GetFlag(mFlags, kFlag_UseCustomDeviceName))
    {
        // FIXME
        // uint16_t discriminator;
        // SuccessOrExit(err = ConfigurationMgr().GetSetupDiscriminator(discriminator));
        uint16_t discriminator = 0xff;
        memset(mDeviceName, 0, kMaxDeviceNameLength);
        snprintf(mDeviceName, kMaxDeviceNameLength, "%s%04u", CHIP_DEVICE_CONFIG_BLE_DEVICE_NAME_PREFIX, discriminator);
    }

    adv_data_builder.setFlags();
    adv_data_builder.setName(mDeviceName);
    mbed_err = ble_interface.gap().setAdvertisingPayload(ble::LEGACY_ADVERTISING_HANDLE, adv_data_builder.getAdvertisingData());
    VerifyOrExit(mbed_err == BLE_ERROR_NONE, err = CHIP_ERROR_INTERNAL);

    mbed_err = ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    VerifyOrExit(mbed_err == BLE_ERROR_NONE, err = CHIP_ERROR_INTERNAL);
#if _BLEMGRIMPL_USE_LEDS
    led3 = 0;
#endif

exit:
    if (mbed_err != BLE_ERROR_NONE)
    {
        ChipLogError(DeviceLayer, "StartAdvertising mbed-os error: %d", mbed_err);
    }
    return err;
}

CHIP_ERROR BLEManagerImpl::StopAdvertising(void)
{
    CHIP_ERROR err       = CHIP_NO_ERROR;
    ble_error_t mbed_err = BLE_ERROR_NONE;

    ble::BLE & ble_interface = ble::BLE::Instance();

    if (!ble_interface.gap().isAdvertisingActive(ble::LEGACY_ADVERTISING_HANDLE))
    {
        ChipLogDetail(DeviceLayer, "No need to stop. Advertising inactive.");
        return err;
    }
    mbed_err = ble_interface.gap().stopAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    VerifyOrExit(mbed_err == BLE_ERROR_NONE, err = CHIP_ERROR_INTERNAL);
#if _BLEMGRIMPL_USE_LEDS
    led3 = 1;
#endif

exit:
    if (mbed_err != BLE_ERROR_NONE)
    {
        ChipLogError(DeviceLayer, "StopAdvertising mbed-os error: %d", mbed_err);
    }
    return err;
}

CHIP_ERROR BLEManagerImpl::_SetCHIPoBLEServiceMode(CHIPoBLEServiceMode val)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(val != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_INVALID_ARGUMENT);
    VerifyOrExit(mServiceMode != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_UNSUPPORTED_CHIP_FEATURE);

    if (val != mServiceMode)
    {
        mServiceMode = val;
        PlatformMgr().ScheduleWork(DriveBLEState, 0);
    }

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::_SetAdvertisingEnabled(bool val)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(mServiceMode != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_UNSUPPORTED_CHIP_FEATURE);

    if (GetFlag(mFlags, kFlag_AdvertisingEnabled) != val)
    {
        ChipLogDetail(DeviceLayer, "SetAdvertisingEnabled(%s)", val ? "true" : "false");

        SetFlag(mFlags, kFlag_AdvertisingEnabled, val);
        PlatformMgr().ScheduleWork(DriveBLEState, 0);
    }

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::_SetFastAdvertisingEnabled(bool val)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(mServiceMode == ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_UNSUPPORTED_CHIP_FEATURE);

    if (GetFlag(mFlags, kFlag_FastAdvertisingEnabled) != val)
    {
        ChipLogDetail(DeviceLayer, "SetFastAdvertisingEnabled(%s)", val ? "true" : "false");

        SetFlag(mFlags, kFlag_FastAdvertisingEnabled, val);
        PlatformMgr().ScheduleWork(DriveBLEState, 0);
    }

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::_GetDeviceName(char * buf, size_t bufSize)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(strlen(mDeviceName) < bufSize, err = CHIP_ERROR_BUFFER_TOO_SMALL);
    strcpy(buf, mDeviceName);

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::_SetDeviceName(const char * deviceName)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(mServiceMode != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_UNSUPPORTED_CHIP_FEATURE);

    if (deviceName != nullptr && deviceName[0] != '\0')
    {
        VerifyOrExit(strlen(deviceName) < kMaxDeviceNameLength, err = CHIP_ERROR_INVALID_ARGUMENT);
        strcpy(mDeviceName, deviceName);
        SetFlag(mFlags, kFlag_UseCustomDeviceName);
        ChipLogDetail(DeviceLayer, "Device name set to: %s", deviceName);
    }
    else
    {
        mDeviceName[0] = '\0';
        ClearFlag(mFlags, kFlag_UseCustomDeviceName);
    }

exit:
    return err;
}

uint16_t BLEManagerImpl::_NumConnections(void)
{
    return mGAPConns;
}

// TODO
void BLEManagerImpl::_OnPlatformEvent(const ChipDeviceEvent * event)
{
    switch (event->Type)
    {
    case DeviceEventType::kCHIPoBLESubscribe: {
        ChipDeviceEvent connEstEvent;

        ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLESubscribe");
        HandleSubscribeReceived(event->CHIPoBLESubscribe.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_TX);
        connEstEvent.Type = DeviceEventType::kCHIPoBLEConnectionEstablished;
        PlatformMgr().PostEvent(&connEstEvent);
    }
    break;

    case DeviceEventType::kCHIPoBLEUnsubscribe: {
        ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEUnsubscribe");
        HandleUnsubscribeReceived(event->CHIPoBLEUnsubscribe.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_TX);
    }
    break;

    case DeviceEventType::kCHIPoBLEWriteReceived: {
        ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEWriteReceived");
        HandleWriteReceived(event->CHIPoBLEWriteReceived.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_RX,
                            PacketBufferHandle::Adopt(event->CHIPoBLEWriteReceived.Data));
    }
    break;

    case DeviceEventType::kCHIPoBLEConnectionError: {
        ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEConnectionError");
        HandleConnectionError(event->CHIPoBLEConnectionError.ConId, event->CHIPoBLEConnectionError.Reason);
    }
    break;

    case DeviceEventType::kCHIPoBLEIndicateConfirm: {
        ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEIndicateConfirm");
        HandleIndicationConfirmation(event->CHIPoBLEIndicateConfirm.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_TX);
    }
    break;

    default:
        ChipLogProgress(DeviceLayer, "_OnPlatformEvent default:  event->Type = %d", event->Type);
        break;
    }
}

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip

#endif // CHIP_DEVICE_CONFIG_ENABLE_CHIPOBLE

/*
 * Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  */

//#include "sdkcommon.h"
//#include "hal/abs_rxtx.h"
//#include "hal/thread.h"
//#include "hal/types.h"
//#include "hal/assert.h"
//#include "hal/locker.h"
//#include "hal/socket.h"
//#include "hal/event.h"

#include "sl_lidar_driver.h"
#include "sl_crc.h" 
#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>

using namespace std;

#ifdef _WIN32
#define NOMINMAX
#undef min
#undef max
#endif

#define DEFAULT_TIMEOUT 100

#if defined(__cplusplus) && __cplusplus >= 201103L
#ifndef _GXX_NULLPTR_T
#define _GXX_NULLPTR_T
typedef decltype(nullptr) nullptr_t;
#endif
#endif /* C++11.  */

namespace sl {
static void printDeprecationWarn(const char* fn, const char* replacement)
{
    fprintf(stderr, "*WARN* YOU ARE USING DEPRECATED API: %s, PLEASE MOVE TO %s\n", fn, replacement);
}


class SlamtecLidarDriver :public ILidarDriver
{
public:
    enum {
        LEGACY_SAMPLE_DURATION = 476,
    };

    enum {
        NORMAL_CAPSULE = 0,
        DENSE_CAPSULE = 1,
    };

    enum {
        A2A3_LIDAR_MINUM_MAJOR_ID  = 2,
        TOF_LIDAR_MINUM_MAJOR_ID = 6,
    };

public:
    SlamtecLidarDriver()
        : _channel(NULL)
        , _isConnected(false)
        , _isScanning(false)
        , _isSupportingMotorCtrl(MotorCtrlSupportNone)
        , _cached_sampleduration_std(LEGACY_SAMPLE_DURATION)
        , _cached_sampleduration_express(LEGACY_SAMPLE_DURATION)

    {}

    //        sl_result connect(IChannel* channel)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;

    //            if (!channel) return SL_RESULT_OPERATION_FAIL;
    //            if (isConnected()) return SL_RESULT_ALREADY_DONE;
    //            _channel = channel;

    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                ans = _channel->open();
    //                if (!ans)
    //                    return SL_RESULT_OPERATION_FAIL;

    //                _channel->flush();
    //            }

    //            _isConnected = true;

    //            ans =checkMotorCtrlSupport(_isSupportingMotorCtrl,500);
    //            return SL_RESULT_OK;

    //        }

    //        void disconnect()
    //        {
    //            if (_isConnected)
    //                _channel->close();
    //        }

    //        bool isConnected()
    //        {
    //            return _isConnected;
    //        }

    //        sl_result reset(sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                ans = _sendCommand(SL_LIDAR_CMD_RESET);
    //                if (!ans) {
    //                    return ans;
    //                }
    //            }
    //            return SL_RESULT_OK;
    //        }

    //        sl_result getAllSupportedScanModes(std::vector<LidarScanMode>& outModes, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            bool confProtocolSupported = false;
    //            ans = checkSupportConfigCommands(confProtocolSupported);
    //            if (!ans) return SL_RESULT_INVALID_DATA;

    //            if (confProtocolSupported) {
    //                // 1. get scan mode count
    //                sl_u16 modeCount;
    //                ans = getScanModeCount(modeCount);
    //                if (!ans) return ans;
    //                // 2. for loop to get all fields of each scan mode
    //                for (sl_u16 i = 0; i < modeCount; i++) {
    //                    LidarScanMode scanModeInfoTmp;
    //                    memset(&scanModeInfoTmp, 0, sizeof(scanModeInfoTmp));
    //                    scanModeInfoTmp.id = i;
    //                    ans = getLidarSampleDuration(scanModeInfoTmp.us_per_sample, i);
    //                    if (!ans) return ans;
    //                    ans = getMaxDistance(scanModeInfoTmp.max_distance, i);
    //                    if (!ans) return ans;
    //                    ans = getScanModeAnsType(scanModeInfoTmp.ans_type, i);
    //                    if (!ans) return ans;
    //                    ans = getScanModeName(scanModeInfoTmp.scan_mode, i);
    //                    if (!ans) return ans;
    //                    outModes.push_back(scanModeInfoTmp);

    //                }
    //                return ans;
    //            }

    //            return ans;
    //        }

    //        sl_result getTypicalScanMode(sl_u16& outMode, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            std::vector<sl_u8> answer;
    //            bool lidarSupportConfigCmds = false;
    //            ans = checkSupportConfigCommands(lidarSupportConfigCmds);
    //            if (!ans) return ans;

    //            if (lidarSupportConfigCmds) {
    //                ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_TYPICAL, answer, std::vector<sl_u8>(), timeoutInMs);
    //                if (!ans) return ans;
    //                if (answer.size() < sizeof(sl_u16)) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }
    //                const sl_u16 *p_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
    //                outMode = *p_answer;
    //                return ans;
    //            }
    //            //old version of triangle lidar
    //            else {
    //                outMode = SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS;
    //                return ans;
    //            }
    //            return ans;
    
    //        }

    //        sl_result startScan(bool force, bool useTypicalScan, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            bool ifSupportLidarConf = false;
    //            startMotor();
    //            ans = checkSupportConfigCommands(ifSupportLidarConf);
    //            if (!ans) return ans;
    //            if (useTypicalScan){
    //                sl_u16 typicalMode;
    //                ans = getTypicalScanMode(typicalMode);
    //                if (!ans) return ans;

    //                //call startScanExpress to do the job
    //                return startScanExpress(false, typicalMode, 0, outUsedScanMode);
    //            }

    //            // 'useTypicalScan' is false, just use normal scan mode
    //            if (ifSupportLidarConf) {
    //                if (outUsedScanMode) {
    //                    outUsedScanMode->id = SL_LIDAR_CONF_SCAN_COMMAND_STD;
    //                    ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
    //                    if (!ans) return ans;
    //                    ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
    //                    if (!ans) return ans;
    //                    ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
    //                    if (!ans) return ans;
    //                    ans = getScanModeName(outUsedScanMode->scan_mode, outUsedScanMode->id);
    //                    if (!ans) return ans;
    //                }
    //            }

    //            return startScanNormal(force);
    //        }

    //        sl_result startScanNormal(bool force, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            if (!isConnected()) return SL_RESULT_OPERATION_FAIL;
    //            if (_isScanning) return SL_RESULT_ALREADY_DONE;

    //            stop(); //force the previous operation to stop
    //            setMotorSpeed();
    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                ans = _sendCommand(force ? SL_LIDAR_CMD_FORCE_SCAN : SL_LIDAR_CMD_SCAN);
    //                if (!ans) return ans;
    //                // waiting for confirmation
    //                sl_lidar_ans_header_t response_header;
    //                ans = _waitResponseHeader(&response_header, timeout);
    //                if (!ans) return ans;

    //                // verify whether we got a correct header
    //                if (response_header.type != SL_LIDAR_ANS_TYPE_MEASUREMENT) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }

    //                sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
    //                if (header_size < sizeof(sl_lidar_response_measurement_node_t)) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }
    //                _isScanning = true;
    //                _cachethread = CLASS_THREAD(SlamtecLidarDriver, _cacheScanData);
    //                if (_cachethread.getHandle() == 0) {
    //                    return SL_RESULT_OPERATION_FAIL;
    //                }
    //            }
    //            return SL_RESULT_OK;
    //        }

    //        sl_result startScanExpress(bool force, sl_u16 scanMode, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            if (!isConnected()) return SL_RESULT_OPERATION_FAIL;
    //            if (_isScanning) return SL_RESULT_ALREADY_DONE;
    //            stop(); //force the previous operation to stop

    //            bool ifSupportLidarConf = false;
    //            ans = checkSupportConfigCommands(ifSupportLidarConf);
    //            if (!ans) return SL_RESULT_INVALID_DATA;


    //            if (outUsedScanMode) {
    //                outUsedScanMode->id = scanMode;
    //                if (ifSupportLidarConf) {
    //                    ans = getLidarSampleDuration(outUsedScanMode->us_per_sample, outUsedScanMode->id);
    //                    if (!ans) return SL_RESULT_INVALID_DATA;

    //                    ans = getMaxDistance(outUsedScanMode->max_distance, outUsedScanMode->id);
    //                    if (!ans) return SL_RESULT_INVALID_DATA;

    //                    ans = getScanModeAnsType(outUsedScanMode->ans_type, outUsedScanMode->id);
    //                    if (!ans) return SL_RESULT_INVALID_DATA;

    //                    ans = getScanModeName(outUsedScanMode->scan_mode, outUsedScanMode->id);
    //                    if (!ans) return SL_RESULT_INVALID_DATA;


    //                }

    //            }

    //            //get scan answer type to specify how to wait data
    //            sl_u8 scanAnsType;
    //            if (ifSupportLidarConf) {
    //                getScanModeAnsType(scanAnsType, scanMode);
    //            }
    //            else {
    //                scanAnsType = SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED;
    //            }
    //            if (!ifSupportLidarConf || scanAnsType == SL_LIDAR_ANS_TYPE_MEASUREMENT) {
    //                if (scanMode == SL_LIDAR_CONF_SCAN_COMMAND_STD) {
    //                    return startScan(force, false, 0, outUsedScanMode);
    //                }
    //            }
    //            {
    //                rp::hal::AutoLocker l(_lock);

    //                startMotor();
    //                sl_lidar_payload_express_scan_t scanReq;
    //                memset(&scanReq, 0, sizeof(scanReq));
    //                if (!ifSupportLidarConf){
    //                    if (scanMode != SL_LIDAR_CONF_SCAN_COMMAND_STD && scanMode != SL_LIDAR_CONF_SCAN_COMMAND_EXPRESS)
    //                        scanReq.working_mode = sl_u8(scanMode);
    //                }else
    //                    scanReq.working_mode = sl_u8(scanMode);

    //                scanReq.working_flags = options;
    //				delay(5);
    //                ans = _sendCommand(SL_LIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq));
    //                if (!ans) {
    //                    ans = _sendCommand(SL_LIDAR_CMD_EXPRESS_SCAN, &scanReq, sizeof(scanReq));
    //                    if (!ans)
    //                        return SL_RESULT_INVALID_DATA;
    //                }


    //                // waiting for confirmation
    //                sl_lidar_ans_header_t response_header;
    //                ans = _waitResponseHeader(&response_header, timeout);
    //                if (!ans) return ans;

    //                // verify whether we got a correct header
    //                if (response_header.type != scanAnsType) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }

    //                sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);

    //                if (_cachethread.getHandle() == 0) {
    //                    return SL_RESULT_OPERATION_FAIL;
    //                }
    //            }
    //            return SL_RESULT_OK;

    //        }

    //        sl_result stop(sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            _disableDataGrabbing();

    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                ans = _sendCommand(SL_LIDAR_CMD_STOP);
    //                if (!ans) return ans;
    //            }
    //            delay(100);

    //            if(_isSupportingMotorCtrl == MotorCtrlSupportPwm)
    //                setMotorSpeed(0);

    //            return SL_RESULT_OK;
    //        }


    //        sl_result grabScanDataHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            switch (_dataEvt.wait(timeout))
    //            {
    //            case rp::hal::Event::EVENT_TIMEOUT:
    //                count = 0;
    //                return SL_RESULT_OPERATION_TIMEOUT;
    //            case rp::hal::Event::EVENT_OK:
    //            {
    //                if (_cached_scan_node_hq_count == 0) return SL_RESULT_OPERATION_TIMEOUT; //consider as timeout

    //                rp::hal::AutoLocker l(_lock);

    //                size_t size_to_copy = std::min(count, _cached_scan_node_hq_count);
    //                memcpy(nodebuffer, _cached_scan_node_hq_buf, size_to_copy * sizeof(sl_lidar_response_measurement_node_hq_t));

    //                count = size_to_copy;
    //                _cached_scan_node_hq_count = 0;
    //            }
    //            return SL_RESULT_OK;

    //            default:
    //                count = 0;
    //                return SL_RESULT_OPERATION_FAIL;
    //            }
    //        }

    //        sl_result getDeviceInfo(sl_lidar_response_device_info_t& info, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            _disableDataGrabbing();
    //			delay(20);
    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                ans = _sendCommand(SL_LIDAR_CMD_GET_DEVICE_INFO);
    //                if (!ans) return ans;
    //                return _waitResponse(info, SL_LIDAR_ANS_TYPE_DEVINFO);
    //            }
    //        }

    //        sl_result checkMotorCtrlSupport(MotorCtrlSupport & support, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            support = MotorCtrlSupportNone;
    //            _disableDataGrabbing();
    //            {
    //                sl_lidar_response_device_info_t devInfo;
    //                ans = getDeviceInfo(devInfo, 500);
    //                if (!ans) return ans;
    //                sl_u8 majorId = devInfo.model >> 4;
    //                if (majorId >= TOF_LIDAR_MINUM_MAJOR_ID) {
    //                        support = MotorCtrlSupportRpm;
    //                        return ans;
    //                }
    //                else if(majorId >= A2A3_LIDAR_MINUM_MAJOR_ID){

    //                    rp::hal::AutoLocker l(_lock);
    //                    sl_lidar_payload_acc_board_flag_t flag;
    //                    flag.reserved = 0;
    //                    ans = _sendCommand(SL_LIDAR_CMD_GET_ACC_BOARD_FLAG, &flag, sizeof(flag));
    //                    if (!ans) return ans;

    //                    sl_lidar_response_acc_board_flag_t acc_board_flag;
    //                    ans = _waitResponse(acc_board_flag, SL_LIDAR_ANS_TYPE_ACC_BOARD_FLAG);
    //                    if (acc_board_flag.support_flag & SL_LIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK) {
    //                        support = MotorCtrlSupportPwm;
    //                    }
    //                    return ans;
    //                }

    //            }
    //            return SL_RESULT_OK;

    //        }

    //        sl_result getFrequency(const LidarScanMode& scanMode, const sl_lidar_response_measurement_node_hq_t* nodes, size_t count, float& frequency)
    //        {
    //            float sample_duration = scanMode.us_per_sample;
    //            frequency = 1000000.0f / (count * sample_duration);
    //            return SL_RESULT_OK;
    //        }

    //		sl_result setLidarIpConf(const sl_lidar_ip_conf_t& conf, sl_u32 timeout)
    //		{
    //			sl_result ans = setLidarConf(SL_LIDAR_CONF_LIDAR_STATIC_IP_ADDR, &conf, sizeof(sl_lidar_ip_conf_t), timeout);
    //			return ans;
    //		}

    //        sl_result getHealth(sl_lidar_response_device_health_t& health, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;

    //            if (!isConnected())
    //                return SL_RESULT_OPERATION_FAIL;

    //            _disableDataGrabbing();

    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                ans = _sendCommand(SL_LIDAR_CMD_GET_DEVICE_HEALTH);
    //                if (!ans) return ans;
    //                delay(50);
    //                ans = _waitResponse(health, SL_LIDAR_ANS_TYPE_DEVHEALTH);

    //            }
    //            return ans;

    //        }

    //		sl_result getDeviceMacAddr(sl_u8* macAddrArray, sl_u32 timeoutInMs)
    //		{
    //			Result<nullptr_t> ans = SL_RESULT_OK;

    //			std::vector<sl_u8> answer;
    //			ans = getLidarConf(SL_LIDAR_CONF_LIDAR_MAC_ADDR, answer, std::vector<sl_u8>(), timeoutInMs);
    //			if (!ans) return ans;

    //			int len = answer.size();
    //			if (0 == len) return SL_RESULT_INVALID_DATA;
    //			memcpy(macAddrArray, &answer[0], len);
    //			return ans;
    //		}

    sl_result ascendScanData(sl_lidar_response_measurement_node_hq_t * nodebuffer, size_t count)
    {
        throw logic_error("NotImple");
//        return ::ascendScanData_<sl_lidar_response_measurement_node_hq_t>(nodebuffer, count);
    }

    //        sl_result getScanDataWithIntervalHq(sl_lidar_response_measurement_node_hq_t * nodebuffer, size_t & count)
    //        {
    //            size_t size_to_copy = 0;
    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                if (_cached_scan_node_hq_count_for_interval_retrieve == 0) {
    //                    return SL_RESULT_OPERATION_TIMEOUT;
    //                }
    //                //copy all the nodes(_cached_scan_node_count_for_interval_retrieve nodes) in _cached_scan_node_buf_for_interval_retrieve
    //                size_to_copy = _cached_scan_node_hq_count_for_interval_retrieve;
    //                memcpy(nodebuffer, _cached_scan_node_hq_buf_for_interval_retrieve, size_to_copy * sizeof(sl_lidar_response_measurement_node_hq_t));
    //                _cached_scan_node_hq_count_for_interval_retrieve = 0;
    //            }
    //            count = size_to_copy;

    //            return SL_RESULT_OK;
    //        }

    //        sl_result setMotorSpeed(sl_u16 speed = DEFAULT_MOTOR_SPEED)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;

    //            if(speed == DEFAULT_MOTOR_SPEED){
    //                sl_lidar_response_desired_rot_speed_t desired_speed;
    //                ans = getDesiredSpeed(desired_speed);
    //                if (!ans) return ans;
    //                if(_isSupportingMotorCtrl == MotorCtrlSupportPwm)
    //                    speed = desired_speed.pwm_ref;
    //                else
    //                    speed = desired_speed.rpm;
    //            }
    //            switch (_isSupportingMotorCtrl)
    //            {
    //            case MotorCtrlSupportNone:
    //                break;
    //            case MotorCtrlSupportPwm:
    //                sl_lidar_payload_motor_pwm_t motor_pwm;
    //                motor_pwm.pwm_value = speed;
    //                ans = _sendCommand(SL_LIDAR_CMD_SET_MOTOR_PWM, (const sl_u8 *)&motor_pwm, sizeof(motor_pwm));
    //                if (!ans) return ans;
    //                break;
    //            case MotorCtrlSupportRpm:
    //                sl_lidar_payload_motor_pwm_t motor_rpm;
    //                motor_rpm.pwm_value = speed;
    //                ans = _sendCommand(SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL, (const sl_u8 *)&motor_rpm, sizeof(motor_rpm));
    //                if (!ans) return ans;
    //                break;
    //            }
    //            return SL_RESULT_OK;
    //        }

    //        sl_result getMotorInfo(LidarMotorInfo &motorInfo, sl_u32 timeoutInMs)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            rp::hal::AutoLocker l(_lock);
    //            {
    //                std::vector<sl_u8> answer;

    //			    ans = getLidarConf(RPLIDAR_CONF_MIN_ROT_FREQ, answer, std::vector<sl_u8>());
    //			    if (!ans) return ans;

    //			    const sl_u16 *min_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
    //                motorInfo.min_speed = *min_answer;


    //                ans = getLidarConf(RPLIDAR_CONF_MAX_ROT_FREQ, answer, std::vector<sl_u8>());
    //			    if (!ans) return ans;

    //			    const sl_u16 *max_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
    //                motorInfo.max_speed = *max_answer;

    //                sl_lidar_response_desired_rot_speed_t desired_speed;
    //                ans = getDesiredSpeed(desired_speed);
    //                if (!ans) return ans;
    //                if(motorInfo.motorCtrlSupport == MotorCtrlSupportPwm)
    //                    motorInfo.desired_speed = desired_speed.pwm_ref;
    //                else
    //                    motorInfo.desired_speed = desired_speed.rpm;

    //            }
    //            return SL_RESULT_OK;
    //        }

protected:
    //        sl_result startMotor()
    //        {
    //            return setMotorSpeed(600);
    //        }

    //        sl_result getDesiredSpeed(sl_lidar_response_desired_rot_speed_t & motorSpeed, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            std::vector<sl_u8> answer;
    //            ans = getLidarConf(SL_LIDAR_CONF_DESIRED_ROT_FREQ, answer, std::vector<sl_u8>(), timeoutInMs);

    //            if (!ans) return ans;

    //            const sl_lidar_response_desired_rot_speed_t *p_answer = reinterpret_cast<const sl_lidar_response_desired_rot_speed_t*>(&answer[0]);
    //            motorSpeed = *p_answer;
    //            return SL_RESULT_OK;
    //        }

    //        sl_result checkSupportConfigCommands(bool& outSupport, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            sl_lidar_response_device_info_t devinfo;
    //            ans = getDeviceInfo(devinfo, timeoutInMs);
    //            if (!ans) return ans;

    //            sl_u16 modecount;
    //            ans = getScanModeCount(modecount, 250);
    //            if ((sl_result)ans == SL_RESULT_OK)
    //                outSupport = true;

    //            return SL_RESULT_OK;
    //        }

    //        sl_result getScanModeCount(sl_u16& modeCount, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            std::vector<sl_u8> answer;
    //            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_COUNT, answer, std::vector<sl_u8>(), timeoutInMs);

    //            if (!ans) return ans;

    //            const sl_u16 *p_answer = reinterpret_cast<const sl_u16*>(&answer[0]);
    //            modeCount = *p_answer;
    //            return SL_RESULT_OK;
    //        }

    //		sl_result setLidarConf(sl_u32 type, const void* payload, size_t payloadSize, sl_u32 timeout)
    //		{
    //			if (type < 0x00010000 || type >0x0001FFFF)
    //				return SL_RESULT_INVALID_DATA;
    //			std::vector<sl_u8> requestPkt;
    //			requestPkt.resize(sizeof(sl_lidar_payload_set_scan_conf_t) + payloadSize);
    //			if (!payload) payloadSize = 0;
    //			sl_lidar_payload_set_scan_conf_t* query = reinterpret_cast<sl_lidar_payload_set_scan_conf_t*>(&requestPkt[0]);

    //			query->type = type;

    //			if (payloadSize)
    //				memcpy(&query[1], payload, payloadSize);

    //			sl_result ans;
    //			{
    //				rp::hal::AutoLocker l(_lock);
    //				if (IS_FAIL(ans = _sendCommand(SL_LIDAR_CMD_SET_LIDAR_CONF, &requestPkt[0], requestPkt.size()))) {//
    //					return ans;
    //				}
    //				delay(20);
    //				// waiting for confirmation
    //				sl_lidar_ans_header_t response_header;
    //				if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
    //					return ans;
    //				}
    //				// verify whether we got a correct header
    //				if (response_header.type != SL_LIDAR_ANS_TYPE_SET_LIDAR_CONF) {
    //					return SL_RESULT_INVALID_DATA;
    //				}
    //				sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
    //				if (header_size < sizeof(type)) {
    //					return SL_RESULT_INVALID_DATA;
    //				}
    //				if (!_channel->waitForData(header_size, timeout)) {
    //					return SL_RESULT_OPERATION_TIMEOUT;
    //				}
    //				delay(100);
    //				struct _sl_lidar_response_set_lidar_conf {
    //					sl_u32 type;
    //					sl_u32 result;
    //				} answer;

    //				_channel->read(reinterpret_cast<sl_u8*>(&answer), header_size);
    //				return answer.result;
    
    //			}

    //		}

    //        sl_result getLidarConf(sl_u32 type, std::vector<sl_u8> &outputBuf, const std::vector<sl_u8> &reserve = std::vector<sl_u8>(), sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            sl_lidar_payload_get_scan_conf_t query;
    //            query.type = type;
    //            int sizeVec = reserve.size();

    //            int maxLen = sizeof(query.reserved) / sizeof(query.reserved[0]);
    //            if (sizeVec > maxLen) sizeVec = maxLen;

    //            if (sizeVec > 0)
    //                memcpy(query.reserved, &reserve[0], reserve.size());

    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            {
    //                rp::hal::AutoLocker l(_lock);
    //                ans = _sendCommand(SL_LIDAR_CMD_GET_LIDAR_CONF, &query, sizeof(query));
    //                if (!ans) return ans;
    //				//delay(50);
    //                // waiting for confirmation
    //                sl_lidar_ans_header_t response_header;
    //                ans = _waitResponseHeader(&response_header, timeout);
    //                if (!ans)return ans;

    //                // verify whether we got a correct header
    //                if (response_header.type != SL_LIDAR_ANS_TYPE_GET_LIDAR_CONF) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }

    //                sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
    //                if (header_size < sizeof(type)) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }
    //				//delay(100);
    //                if (!_channel->waitForData(header_size, timeout)) {
    //                    return SL_RESULT_OPERATION_TIMEOUT;
    //                }

    //                std::vector<sl_u8> dataBuf;
    //                dataBuf.resize(header_size);
    //                _channel->read(reinterpret_cast<sl_u8 *>(&dataBuf[0]), header_size);

    //                //check if returned type is same as asked type
    //                sl_u32 replyType = -1;
    //                memcpy(&replyType, &dataBuf[0], sizeof(type));
    //                if (replyType != type) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }

    //                //copy all the payload into &outputBuf
    //                int payLoadLen = header_size - sizeof(type);

    //                //do consistency check
    //                if (payLoadLen <= 0) {
    //                    return SL_RESULT_INVALID_DATA;
    //                }
    //                //copy all payLoadLen bytes to outputBuf
    //                outputBuf.resize(payLoadLen);
    //                memcpy(&outputBuf[0], &dataBuf[0] + sizeof(type), payLoadLen);

    //            }

    //            return SL_RESULT_OK;
    //        }

    //        sl_result getLidarSampleDuration(float& sampleDurationRes, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            std::vector<sl_u8> reserve(2);
    //            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    //            std::vector<sl_u8> answer;
    //            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, answer, reserve, timeoutInMs);

    //            if (!ans) return ans;

    //            if (answer.size() < sizeof(sl_u32)){
    //                return SL_RESULT_INVALID_DATA;
    //            }
    //            const sl_u32 *result = reinterpret_cast<const sl_u32*>(&answer[0]);
    //            sampleDurationRes = (float)(*result >> 8);
    //            return ans;
    //        }

    //        sl_result getMaxDistance(float &maxDistance, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            std::vector<sl_u8> reserve(2);
    //            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    //            std::vector<sl_u8> answer;
    //            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_MAX_DISTANCE, answer, reserve, timeoutInMs);
    //            if (!ans) return ans;

    //            if (answer.size() < sizeof(sl_u32)){
    //                return SL_RESULT_INVALID_DATA;
    //            }
    //            const sl_u32 *result = reinterpret_cast<const sl_u32*>(&answer[0]);
    //            maxDistance = (float)(*result >> 8);
    //            return ans;
    //        }

    //        sl_result getScanModeAnsType(sl_u8 &ansType, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            std::vector<sl_u8> reserve(2);
    //            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    //            std::vector<sl_u8> answer;
    //            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_ANS_TYPE, answer, reserve, timeoutInMs);
    //            if (!ans) return ans;

    //            if (answer.size() < sizeof(sl_u8)){
    //                return SL_RESULT_INVALID_DATA;
    //            }
    //            const sl_u8 *result = reinterpret_cast<const _u8*>(&answer[0]);
    //            ansType = *result;
    //            return ans;

    //        }

    //        sl_result getScanModeName(char* modeName, sl_u16 scanModeID, sl_u32 timeoutInMs = DEFAULT_TIMEOUT)
    //        {
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            std::vector<sl_u8> reserve(2);
    //            memcpy(&reserve[0], &scanModeID, sizeof(scanModeID));

    //            std::vector<sl_u8> answer;
    //            ans = getLidarConf(SL_LIDAR_CONF_SCAN_MODE_NAME, answer, reserve, timeoutInMs);
    //            if (!ans) return ans;
    //            int len = answer.size();
    //            if (0 == len) return SL_RESULT_INVALID_DATA;
    //            memcpy(modeName, &answer[0], len);
    //            return ans;
    //        }

    //        sl_result negotiateSerialBaudRate(sl_u32 requiredBaudRate, sl_u32 * baudRateDetected)
    //        {
    //            // ask the LIDAR to stop working first...
    //            stop();
    //            _channel->flush();

    //            // wait for a while
    //            delay(10);
    //            _channel->clearReadCache();

    //            // sending magic byte to let the target LIDAR start baudrate measurement
    //            // More than 100 bytes per second datarate is required to trigger the measurements
    //            {


    //                sl_u8 magicByteSeq[16];

    //                memset(magicByteSeq, SL_LIDAR_AUTOBAUD_MAGICBYTE, sizeof(magicByteSeq));

    //                sl_u64 startTS = getms();

    //                while (getms() - startTS < 1500 ) //lasting for 1.5sec
    //                {
    //                    if (_channel->write(magicByteSeq, sizeof(magicByteSeq)) < 0)
    //                    {
    //                        return SL_RESULT_OPERATION_FAIL;
    //                    }

    //                    size_t dataCountGot;
    //                    if (_channel->waitForData(1, 1, &dataCountGot)) {
    //                        //got reply, stop
    //                        break;
    //                    }
    //                }
    //            }

    //            // getback the bps measured
    //            _u32 bpsDetected = 0;
    //            size_t dataCountGot;
    //            if (_channel->waitForData(4, 500, &dataCountGot)) {
    //                //got reply, stop
    //                _channel->read(&bpsDetected, 4);
    //                if (baudRateDetected) *baudRateDetected = bpsDetected;


    //                // send a confirmation to the LIDAR, otherwise, the previous baudrate will be reverted back
    //                sl_lidar_payload_new_bps_confirmation_t confirmation;
    //                confirmation.flag = 0x5F5F;
    //                confirmation.required_bps = requiredBaudRate;
    //                confirmation.param = 0;
    //                if (SL_IS_FAIL(_sendCommand(RPLIDAR_CMD_NEW_BAUDRATE_CONFIRM, &confirmation, sizeof(confirmation))))
    //                    return RESULT_OPERATION_FAIL;


    //                return RESULT_OK;
    //            }

    //            return RESULT_OPERATION_TIMEOUT;
    //        }

private:



    //        sl_result  _sendCommand(sl_u16 cmd, const void * payload = NULL, size_t payloadsize = 0 )
    //        {
    //            sl_u8 pkt_header[10];
    //            sl_u8 checksum = 0;

    //            std::vector<sl_u8> cmd_packet;
    //            cmd_packet.clear();

    //            if (payloadsize && payload) {
    //                cmd |= SL_LIDAR_CMDFLAG_HAS_PAYLOAD;
    //            }
    //			_channel->flush();
    //            cmd_packet.push_back(SL_LIDAR_CMD_SYNC_BYTE);
    //            cmd_packet.push_back(cmd);

    //            if (cmd & SL_LIDAR_CMDFLAG_HAS_PAYLOAD) {
    //                std::vector<sl_u8> payloadMsg;
    //                checksum ^= SL_LIDAR_CMD_SYNC_BYTE;
    //                checksum ^= cmd;
    //                checksum ^= (payloadsize & 0xFF);

    //                // send size
    //                sl_u8 sizebyte = payloadsize;
    //                cmd_packet.push_back(sizebyte);
    //                // calc checksum
    //                for (size_t pos = 0; pos < payloadsize; ++pos) {
    //                    checksum ^= ((sl_u8 *)payload)[pos];
    //                    cmd_packet.push_back(((sl_u8 *)payload)[pos]);
    //                }
    //                cmd_packet.push_back(checksum);

    //            }
    //            sl_u8 packet[1024];
    //            for (int pos = 0; pos < cmd_packet.size(); pos++) {
    //                packet[pos] = cmd_packet[pos];
    //            }
    //            _channel->write(packet, cmd_packet.size());
    //            delay(1);
    //            return SL_RESULT_OK;
    //        }

    //        sl_result _waitResponseHeader(sl_lidar_ans_header_t * header, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            int  recvPos = 0;
    //            sl_u32 startTs = getms();
    //            sl_u8  recvBuffer[sizeof(sl_lidar_ans_header_t)];
    //            sl_u8  *headerBuffer = reinterpret_cast<sl_u8 *>(header);
    //            sl_u32 waitTime;

    //            while ((waitTime = getms() - startTs) <= timeout) {
    //                size_t remainSize = sizeof(sl_lidar_ans_header_t) - recvPos;
    //                size_t recvSize;

    //                bool ans = _channel->waitForData(remainSize, timeout - waitTime, &recvSize);
    //                if (!ans) return SL_RESULT_OPERATION_TIMEOUT;

    //                if (recvSize > remainSize) recvSize = remainSize;

    //                recvSize = _channel->read(recvBuffer, recvSize);

    //                for (size_t pos = 0; pos < recvSize; ++pos) {
    //                    sl_u8 currentByte = recvBuffer[pos];
    //                    switch (recvPos) {
    //                    case 0:
    //                        if (currentByte != SL_LIDAR_ANS_SYNC_BYTE1) {
    //                            continue;
    //                        }

    //                        break;
    //                    case 1:
    //                        if (currentByte != SL_LIDAR_ANS_SYNC_BYTE2) {
    //                            recvPos = 0;
    //                            continue;
    //                        }
    //                        break;
    //                    }
    //                    headerBuffer[recvPos++] = currentByte;

    //                    if (recvPos == sizeof(sl_lidar_ans_header_t)) {
    //                        return SL_RESULT_OK;
    //                    }
    //                }
    //            }

    //            return SL_RESULT_OPERATION_TIMEOUT;
    //        }

    //        template <typename T>
    //        sl_result _waitResponse(T &payload ,sl_u8 ansType, sl_u32 timeout = DEFAULT_TIMEOUT)
    //        {
    //            sl_lidar_ans_header_t response_header;
    //            Result<nullptr_t> ans = SL_RESULT_OK;
    //            //delay(100);
    //            ans = _waitResponseHeader(&response_header, timeout);
    //            if (!ans)
    //                return ans;
    //            // verify whether we got a correct header
    //            if (response_header.type != ansType) {
    //                return SL_RESULT_INVALID_DATA;
    //            }
    //           // delay(50);
    //            sl_u32 header_size = (response_header.size_q30_subtype & SL_LIDAR_ANS_HEADER_SIZE_MASK);
    //            if (header_size < sizeof(T)) {
    //                return SL_RESULT_INVALID_DATA;
    //            }
    //            if (!_channel->waitForData(header_size, timeout)) {
    //                return SL_RESULT_OPERATION_TIMEOUT;
    //            }
    //            _channel->read(reinterpret_cast<sl_u8 *>(&payload), sizeof(T));
    //            return SL_RESULT_OK;
    //        }

    //        void _disableDataGrabbing()
    //        {
    //            //_clearRxDataCache();
    //            _isScanning = false;
    //            _cachethread.join();
    //        }

#define  MAX_SCAN_NODES  (8192)











    sl_result _clearRxDataCache()
    {
        if (!isConnected())
            return SL_RESULT_OPERATION_FAIL;
        _channel->flush();
        return SL_RESULT_OK;
    }

private:
    IChannel *_channel;
    bool _isConnected;
    bool _isScanning;
    MotorCtrlSupport        _isSupportingMotorCtrl;
    //        rp::hal::Locker         _lock;
    //        rp::hal::Event          _dataEvt;
    //        rp::hal::Thread         _cachethread;
    sl_u16                  _cached_sampleduration_std;
    sl_u16                  _cached_sampleduration_express;
    bool                    _scan_node_synced;


    sl_u8                                    _cached_capsule_flag;


    //  sl_lidar_response_capsule_measurement_nodes_t       _cached_previous_capsuledata;
    //   sl_lidar_response_dense_capsule_measurement_nodes_t _cached_previous_dense_capsuledata;
    //   sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
    //   sl_lidar_response_hq_capsule_measurement_nodes_t _cached_previous_Hqdata;
    //    bool                                         _is_previous_capsuledataRdy;
    //    bool                                         _is_previous_HqdataRdy;
};


}


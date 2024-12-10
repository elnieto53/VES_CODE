/*
 * Copyright (c) 2020, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the B105 Electronic Systems Lab.
 * 4. Neither the name of the B105 Electronic Systems Lab nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY UNIVERSITY AND CONTRIBUTORS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * drv2605L.c
 *
 *  Created on: Mar 22, 2020
 *      Author: Manuel Simal Perez <msimal@b105.upm.es>
 */
#include <drv2605L.h>
#include "esp_log.h"

#include "bsp.h"


//osMutexDef (drv260x_mutex);



drv2605L_t* drv2605L_create (i2c_line_t* i2c_line)
{
    /* Allocate memory */
    drv2605L_t* drvDevice = pvPortMalloc(sizeof(drv2605L_t));
    if (drvDevice == NULL){
    	return NULL;
    }

	drvDevice->lock = xSemaphoreCreateMutex();
    drvDevice->state = DRIVER_NOT_INIT;
    drvDevice->i2c_line = i2c_line;

    return drvDevice;
}

retval_t drv2605L_delete (drv2605L_t* drvDevice)
{
    if (drvDevice == NULL)
    {
        return RET_ERROR;
    }

    if (drvDevice->state != DRIVER_NOT_INIT)
    {
        return RET_ERROR;
    }

    /* Delete mutex */
    //osMutexDelete(drvDevice->lock);
    vSemaphoreDelete(drvDevice->lock);

    /* Free memory */
    vPortFree(drvDevice);

    return RET_OK;
}

retval_t drv2605L_init(drv2605L_t* drvDevice, actuatorAddress_t i2cChannel)
{
    if (drvDevice == NULL)
    {
        return RET_ERROR;
    }

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    /* Check for already initialized driver */
    if(drvDevice->state != DRIVER_NOT_INIT)
    {
    	xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    /*Open I2C drvDevice */
    if(drv2605_init(drvDevice->i2c_line) != RET_OK)
    	return RET_ERROR;

//    uint16_t addressSize = 1;

    /* Initialize drvDevice */
    drvDevice->state = DRIVER_READY;
    drvDevice->devAddress = i2cChannel;

    xSemaphoreGive(drvDevice->lock);

    if (drv2605L_reset(drvDevice) != RET_OK)
    {
    	xSemaphoreTake(drvDevice->lock, portMAX_DELAY);
        //ytClose(drvDevice->i2cDevice);
        drvDevice->state = DRIVER_NOT_INIT;
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    return RET_OK;
}

retval_t drv2605L_deinit(drv2605L_t* drvDevice)
{
    if (drvDevice == NULL)
    {
        return RET_ERROR;
    }

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    /* Disable drvDevice */
    //ytGpioPinReset(drvDevice->ENgpio);
    //gpio_set_level(drvDevice->ENgpio, 0);

    /* Close I2C port */
    //ytClose(drvDevice->i2cDevice);

    drvDevice->state = DRIVER_NOT_INIT;

    xSemaphoreGive(drvDevice->lock);
    
    return RET_OK;
}

retval_t drv2605L_reset(drv2605L_t* drvDevice)
{
    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, 0) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    
    xSemaphoreGive(drvDevice->lock);
    return RET_OK;
}

retval_t drv2605L_enterStandby(drv2605L_t* drvDevice)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if (drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    modeReg.b.bStandby = true;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, modeReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);
    return RET_OK;
}

retval_t drv2605L_exitStandby(drv2605L_t* drvDevice)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    modeReg.b.bStandby = false;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, modeReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);
    return RET_OK;
}

retval_t drv2605L_getDeviceID(drv2605L_t* drvDevice,
                    drv2605L_DeviceID_t* id)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

//    *id = (drv2605L_DeviceID_t)statusReg.b.bDeviceID;
    *id = (drv2605L_DeviceID_t)statusReg.r;
    return RET_OK;
}

retval_t drv2605L_setMode(drv2605L_t* drvDevice, drv2605L_Mode_t mode)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    modeReg.b.bMode = (uint8_t)mode;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_mode, modeReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_getMode(drv2605L_t* drvDevice,
                drv2605L_Mode_t* mode)
{
    drv2605L_ModeReg_t modeReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if (drv2605L_register_read(drvDevice, drv2605L_reg_mode, &(modeReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    xSemaphoreGive(drvDevice->lock);

    *mode = (drv2605L_Mode_t)(modeReg.b.bMode);
    return RET_OK;
}

retval_t drv2605L_getDiagResult(drv2605L_t* drvDevice,
                      bool* diagResult)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    *diagResult = statusReg.b.bDiagResult;
    return RET_OK;
}
retval_t drv2605L_getTempStatus(drv2605L_t* drvDevice,
                      bool* tempStatus)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    *tempStatus = statusReg.b.bOverTemp;
    return RET_OK;
}

retval_t drv2605L_getOverCurrentStatus(drv2605L_t* drvDevice,
                             bool* overCurrentStatus)
{
    drv2605L_StatusReg_t statusReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_status, &(statusReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    *overCurrentStatus = statusReg.b.bOverCurrent;
    return RET_OK;
}

retval_t drv2605L_loadActuatorConfig(drv2605L_t* drvDevice, const drv2605L_Actuator_t *actuator)
{
    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if (drv2605L_block_write(drvDevice,
                            drv2605L_reg_ratedVoltage,
                            (uint8_t*)&(actuator->actuatorData[0]),
                            10) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_go(drv2605L_t* drvDevice)
{
    drv2605L_GoReg_t goReg;
    goReg.b.bGo = true;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_write(drvDevice, drv2605L_reg_go, goReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_stop(drv2605L_t* drvDevice)
{
    drv2605L_GoReg_t goReg;
    goReg.b.bGo = false;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_write(drvDevice, drv2605L_reg_go, goReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_blockUntilGoIsCleared(drv2605L_t* drvDevice)
{
    drv2605L_GoReg_t goReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    do
    {
        if(drv2605L_register_read(drvDevice, drv2605L_reg_go, &(goReg.r)) != RET_OK)
        {
            xSemaphoreGive(drvDevice->lock);
            return RET_ERROR;
        }
    }
    while (goReg.b.bGo != false);
    
    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_runAutoCalibration(drv2605L_t* drvDevice,
                           bool* result)
{
    drv2605L_Mode_t contextSavedMode;
    bool diagnosticResult;

    if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_getMode(drvDevice, &contextSavedMode) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_setMode(drvDevice, drv2605L_mode_autoCalibration) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_go(drvDevice) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_getDiagResult(drvDevice, &diagnosticResult) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_setMode(drvDevice, contextSavedMode) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    
    *result = diagnosticResult;
    return RET_OK;
}

retval_t drv2605L_storeAutoCalibrationResults(drv2605L_t* drvDevice, drv2605L_Actuator_t *actuator)
{
    uint8_t compensation, backEMF;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_autoCalibrationCompResult, &compensation) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    if(drv2605L_register_read(drvDevice, drv2605L_reg_autoCalibrationBackEMFResult, &backEMF) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    actuator->autoCal_BackEMF.b.bAutocalBackEMFResult = backEMF;
    actuator->autoCal_Compensation.b.bAutocalCompResult = compensation;
    return RET_OK;
}

retval_t drv2605L_selectEffectLibrary(drv2605L_t* drvDevice, drv2605L_Library_t library)
{
    drv2605L_LibSelectReg_t libSelectReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_librarySelection, &(libSelectReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    libSelectReg.b.bLibrarySelection = (uint8_t)library;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_librarySelection, libSelectReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_fireROMLibraryEffect(drv2605L_t* drvDevice, drv2605L_Effect_t effect, bool wait)
{
    drv2605L_WaveformSeqReg_t playlist[2];

    playlist[0].b.bParam = effect;
    playlist[0].b.bDelay = false;
    playlist[1].b.bParam = drv2605L_effect_stop;
    playlist[1].b.bDelay = false;

    if (wait == true)
    {
        if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
        {
            xSemaphoreGive(drvDevice->lock);
            return RET_ERROR;
        }
    }

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_block_write(drvDevice, drv2605L_reg_waveformSequence1, &(playlist[0].r), 2) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    
    xSemaphoreGive(drvDevice->lock);

    if(drv2605L_go(drvDevice) != RET_OK)
    {
        return RET_ERROR;
    }

    return RET_OK;
}

retval_t drv2605L_setHighZOutputState(drv2605L_t* drvDevice)
{
    drv2605L_LibSelectReg_t libSelectReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_librarySelection, &(libSelectReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    libSelectReg.b.bHighZ = true;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_librarySelection, libSelectReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_clearHighZOutputState(drv2605L_t* drvDevice)
{
    drv2605L_LibSelectReg_t libSelectReg;

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_register_read(drvDevice, drv2605L_reg_librarySelection, &(libSelectReg.r)) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    libSelectReg.b.bHighZ = false;
    if(drv2605L_register_write(drvDevice, drv2605L_reg_librarySelection, libSelectReg.r) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_loadWaveformPlaylist(drv2605L_t* drvDevice,
        const drv2605L_WaveformPlaylist_t *playlist)
{
    if(drv2605L_blockUntilGoIsCleared(drvDevice) != RET_OK)
    {
        return RET_ERROR;
    }

    /* Wait for mutex */
    xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

    if(drv2605L_block_write(drvDevice,
    				       drv2605L_reg_waveformSequence1,
			               (uint8_t*)(&playlist->waveformPlaylistData[0]),
					       8) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }
    if(drv2605L_block_write(drvDevice,
    	                   drv2605L_reg_overdriveTimeOffset,
					       (uint8_t*)(&playlist->waveformPlaylistData[8]),
					       4) != RET_OK)
    {
        xSemaphoreGive(drvDevice->lock);
        return RET_ERROR;
    }

    xSemaphoreGive(drvDevice->lock);

    return RET_OK;
}

retval_t drv2605L_fireWaveformPlaylist(drv2605L_t* drvDevice,
        const drv2605L_WaveformPlaylist_t *playlist)
{
    if(drv2605L_loadWaveformPlaylist(drvDevice, playlist) != RET_OK)
    {
        return RET_ERROR;
    }
    if(drv2605L_go(drvDevice) != RET_OK)
    {
        return RET_ERROR;
    }

    return RET_OK;
}

retval_t drv2605L_setRTPInput(drv2605L_t* drvDevice, uint8_t value){
	drv2605L_RealtimePlaybackInputReg_t rtpReg;
	rtpReg.b.bRTPInput = value;

	/* Wait for mutex */
	xSemaphoreTake(drvDevice->lock, portMAX_DELAY);

	if(drv2605L_register_write(drvDevice, drv2605L_reg_realTimePlaybackInput, rtpReg.r) != RET_OK)
	{
		xSemaphoreGive(drvDevice->lock);
		return RET_ERROR;
	}

	xSemaphoreGive(drvDevice->lock);

	return RET_OK;
}

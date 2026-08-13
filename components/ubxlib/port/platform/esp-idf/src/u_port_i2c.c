/*
 * u_port_i2c.c - STUB I2C port for the UWatch project.
 *
 * The UWatch GNSS receiver is connected via UART, so I2C is never used.
 * ubxlib's real ESP-IDF I2C port uses the legacy driver/i2c.h API, which
 * conflicts with the new i2c_master driver used elsewhere in this project
 * (twatch_board). This stub keeps the GNSS feature linkable without pulling
 * in that legacy driver. All operations return U_ERROR_COMMON_NOT_SUPPORTED.
 */
#include "u_error_common.h"
#include "u_port.h"
#include "u_port_i2c.h"

int32_t uPortI2cInit()
{
    return (int32_t) U_ERROR_COMMON_SUCCESS;
}

void uPortI2cDeinit()
{
}

int32_t uPortI2cOpen(int32_t i2c, int32_t pinSda, int32_t pinSdc,
                     bool controller)
{
    (void) i2c;
    (void) pinSda;
    (void) pinSdc;
    (void) controller;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cAdopt(int32_t i2c, bool controller)
{
    (void) i2c;
    (void) controller;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

void uPortI2cClose(int32_t handle)
{
    (void) handle;
}

int32_t uPortI2cCloseRecoverBus(int32_t handle)
{
    (void) handle;
    return (int32_t) U_ERROR_COMMON_SUCCESS;
}

int32_t uPortI2cSetClock(int32_t handle, int32_t clockHertz)
{
    (void) handle;
    (void) clockHertz;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cGetClock(int32_t handle)
{
    (void) handle;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cSetTimeout(int32_t handle, int32_t timeoutMs)
{
    (void) handle;
    (void) timeoutMs;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cGetTimeout(int32_t handle)
{
    (void) handle;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cControllerExchange(int32_t handle, uint16_t address,
                                   const char *pSend, size_t bytesToSend,
                                   char *pReceive, size_t bytesToReceive,
                                   bool noInterveningStop)
{
    (void) handle;
    (void) address;
    (void) pSend;
    (void) bytesToSend;
    (void) pReceive;
    (void) bytesToReceive;
    (void) noInterveningStop;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cControllerSendReceive(int32_t handle, uint16_t address,
                                      const char *pSend, size_t bytesToSend,
                                      char *pReceive, size_t bytesToReceive)
{
    (void) handle;
    (void) address;
    (void) pSend;
    (void) bytesToSend;
    (void) pReceive;
    (void) bytesToReceive;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cControllerSend(int32_t handle, uint16_t address,
                               const char *pSend, size_t bytesToSend,
                               bool noStop)
{
    (void) handle;
    (void) address;
    (void) pSend;
    (void) bytesToSend;
    (void) noStop;
    return (int32_t) U_ERROR_COMMON_NOT_SUPPORTED;
}

int32_t uPortI2cResourceAllocCount()
{
    return 0;
}

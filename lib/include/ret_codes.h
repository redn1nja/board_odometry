#pragma once

typedef enum {
    RC_OK = 0,                  /**< No error */
    RC_ERROR = -1,              /**< General error */
    RC_ERROR_NULL_PTR = -2,     /**< Null pointer */
    RC_ERROR_INVALID_ARG = -3,  /**< Invalid argument */
    RC_ERROR_INVALID_STATE = -4,/**< Invalid state */
    RC_ERROR_INVALID_OP = -5,   /**< Invalid operation */
    RC_ERROR_NOT_IMPLEMENTED = -6, /**< Not implemented */
    RC_ERROR_NOT_SUPPORTED = -7,    /**< Not supported */
    RC_ERROR_OUT_OF_MEMORY = -8,   /**< Out of memory */
    RC_ERROR_BUSY = -9,             /**< Busy */
    RC_ERROR_TIMEOUT = -10,         /**< Timeout */
    RC_ERROR_IO = -11,              /**< IO error */
    RC_ERROR_NO_DEVICE = -12,       /**< No device */
} rc_t;
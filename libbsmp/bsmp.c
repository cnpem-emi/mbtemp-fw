#include "bsmp.h"

static const char* error_str [BSMP_ERR_MAX] =
{
    "Success",
    "An invalid parameter was passed",
    "A parameter was out of the acceptable range",
    "Not enough memory to complete request",
    "Entity already registered",
    "Sending or receiving a message failed",
    "Instance not initialized",
};

const char *bsmp_error_str (enum bsmp_err error)
{
    return error_str[error];
}

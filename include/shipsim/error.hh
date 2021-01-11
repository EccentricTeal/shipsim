#ifndef SHIPSIM_ERROR_HH
#define SHIPSIM_ERROR_HH

enum class ShipsimStatus: int
{
    READY = 0,
    RUNNING = 1,
    NOT_READY = -1,
    ERROR = -2
};

enum class ShipsimError: int
{
    OK = 0,
    FAILED_TO_RUN = -1,
    INVALID_MODEL = -10
};

#endif
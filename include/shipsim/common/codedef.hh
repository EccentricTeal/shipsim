#ifndef SHIPSIM_ERROR_HH
#define SHIPSIM_ERROR_HH

namespace shipsim::common
{

  enum class Status: int
  {
    READY = 0,
    RUNNING = 1,
    NOT_READY = -1,
    ERROR = -2
  };

  enum class Error: int
  {
    OK = 0,
    FAILED_TO_RUN = -1,
    INVALID_MODEL = -10,
    DIV_BY_ZERO = -20
  };    
    
}


#endif
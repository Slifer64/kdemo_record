#include <kdemo_record/utils.h>

bool makeThreadRT(std::thread &thr)
{
  struct sched_param sch_param;
  sch_param.sched_priority = 99;
  int policy = SCHED_FIFO;
  int ret = pthread_setschedparam(thr.native_handle(), policy, &sch_param);

  if (ret)
  {
    switch (ret)
    {
      std::cerr << "[makeThreadRT::ERROR]:\n ********* ERROR setting thread in RT *********\n===> Reason:\n";
      case ESRCH:
        std::cerr << "No thread with the ID thread could be found.\n";
        break;
      case EINVAL:
        std::cerr << "Policy is not a recognized policy, or param does not make sense for the policy.\n";
        break;
      case EPERM:
        std::cerr << "The caller does not have appropriate privileges to set the specified scheduling policy and parameters.\n";
        break;
      case ENOTSUP:
        std::cerr << "Attempt was made to set the policy or scheduling parameters to an unsupported value.\n";
        break;
      default:
        std::cerr << "Unknown error coce: \"" << ret << "\"\n";
    }
    return false;
  }
  return true;
}

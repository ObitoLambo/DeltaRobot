
//
// File ros2nodeinterface.cpp
//
// Code generated for Simulink model 'sim4_ROS2_delta'.
//
// Model version                  : 1.230
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Thu Jul 02 13:41:51 2026
//
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma comment(lib, "Ws2_32.lib")
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "sim4_ROS2_delta.h"
#include "ros2nodeinterface.h"
#include <thread>
#include <chrono>
#include <utility>
#undef ROS_SET_RTM_ERROR_STATUS
#undef ROS_GET_RTM_ERROR_STATUS
#undef ROS_RTM_STEP_TASK
#define ROS_SET_RTM_ERROR_STATUS(status)  rtmSetErrorStatus(sim4_ROS2_delta_M,(status));
#define ROS_GET_RTM_ERROR_STATUS()        rtmGetErrorStatus(sim4_ROS2_delta_M)
#define ROS_RTM_STEP_TASK(id)             rtmStepTask(sim4_ROS2_delta_M,id)
#include "slros2_multi_threaded_executor.h"
std::vector<rclcpp::SubscriptionBase*> SLROSSubscribers;
extern rclcpp::Node::SharedPtr SLROSNodePtr;
#ifndef RT_MEMORY_ALLOCATION_ERROR_DEF
#define RT_MEMORY_ALLOCATION_ERROR_DEF
const char *RT_MEMORY_ALLOCATION_ERROR = "memory allocation error";
#endif
namespace ros2 {
namespace matlab {
NodeInterface::NodeInterface()
    : mExec()
    , mBaseRateSem()
    , mBaseRateThread()
    , mSchedulerTimer()
    , mExtModeThread()
    , mStopSem()
    , mRunModel(true){
  }
NodeInterface::~NodeInterface() {
    terminate();
  }
void NodeInterface::initialize(int argc, char * const argv[]) {
    try {
        //initialize ros2
        std::vector<char *> args(argv, argv + argc);
        rclcpp::init(static_cast<int>(args.size()), args.data());
        //create the Node specified in Model
        std::string NodeName("sim4_ROS2_delta");
        SLROSNodePtr = std::make_shared<rclcpp::Node>(NodeName);
        RCLCPP_INFO(SLROSNodePtr->get_logger(),"** Starting the model \"sim4_ROS2_delta\" **\n");
        mExec = std::make_shared<rclcpp::executors::SLMultiThreadedExecutor>();
        mExec->add_node(SLROSNodePtr);
        {
			char* extmodeArg[] = {"sim4_ROS2_delta","-port","17725","-verbose","0","-w"};
            errorCode = extmodeParseArgs(6, (const char_T **)extmodeArg);
            if (errorCode != EXTMODE_SUCCESS) {
                RCLCPP_ERROR(SLROSNodePtr->get_logger(),"!!! Error while parsing ExtModeArgs. Error code: %d",errorCode);
            }
		}
        //initialize the model which will initialize the publishers and subscribers
        ROS_SET_RTM_ERROR_STATUS((NULL));
        sim4_ROS2_delta_initialize();
		/* External mode */
        errorCode = extmodeInit(sim4_ROS2_delta_M->extModeInfo, &rtmGetTFinal(sim4_ROS2_delta_M));
        if (errorCode != EXTMODE_SUCCESS) {
            RCLCPP_ERROR(SLROSNodePtr->get_logger(),"!!! Extmode Init failed. Error code: %d",errorCode);
        }
        if (errorCode == EXTMODE_SUCCESS) {
            // Wait until a Start or Stop Request has been received from the Host
            extmodeWaitForHostRequest(EXTMODE_WAIT_FOREVER);
            if (extmodeStopRequested()) {
                rtmSetStopRequested(sim4_ROS2_delta_M, true);
            }
        }
		mExtModeThread = std::make_shared<std::thread>(&NodeInterface::extmodeBackgroundTask, this);
        //create the threads for the rates in the Model
        mBaseRateThread = std::make_shared<std::thread>(&NodeInterface::baseRateTask, this);
        // Create "MutuallyExclusive" callback group for callback associated with mSchedulerTimer to prevent
        // it from being executed in parallel.
		mSchedulerGroup = SLROSNodePtr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		mSchedulerTimer = SLROSNodePtr->create_wall_timer(std::chrono::nanoseconds(200000000),std::bind(&NodeInterface::schedulerThreadCallback,this),mSchedulerGroup);
		for(size_t ctr = 0; ctr<SLROSSubscribers.size();ctr++){
           mExec->stopSubscriberCallback(SLROSSubscribers[ctr]);
        }
    }
    catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
        throw ex;
    }
    catch (...) {
        std::cout << "Unknown exception" << std::endl;
        throw;
    }
}
int NodeInterface::run() {
  if (mExec) {
    mExec->spin();
  }
  mRunModel = false;
  return 0;
}
boolean_T NodeInterface::getStopRequestedFlag(void) {
    #ifndef rtmGetStopRequested
    return (!(ROS_GET_RTM_ERROR_STATUS() == (NULL)));
    #else
    return (!(ROS_GET_RTM_ERROR_STATUS()
        == (NULL)) || rtmGetStopRequested(sim4_ROS2_delta_M));
    #endif
}
void NodeInterface::stop(void) {
  if (mExec.get()) {
    mExec->cancel();
    if (SLROSNodePtr) {
      mExec->remove_node(SLROSNodePtr);
    }
    while (mExec.use_count() > 1);
  }
}
void NodeInterface::terminate(void) {
    if (mBaseRateThread.get()) {
        mRunModel = false;
        mBaseRateSem.notify();
        mBaseRateThread->join();
		if (mSchedulerTimer.get()) {
        	mSchedulerTimer->cancel();
        	mSchedulerTimer->reset();
		}
        mBaseRateThread.reset();
        sim4_ROS2_delta_terminate();
          extmodeReset();
        mExec.reset();
        SLROSNodePtr.reset();
        rclcpp::shutdown();
    }
}
//
// Scheduler Task using clock timer to run base-rate
//
void NodeInterface::schedulerThreadCallback(void)
{
	if(mRunModel) {
        mBaseRateSem.notify();
    }
}
//
//Model specific
// Base-rate task
void NodeInterface::baseRateTask(void) {
  mRunModel = (ROS_GET_RTM_ERROR_STATUS() ==
              (NULL));
  while (mRunModel) {
    mBaseRateSem.wait();
#ifdef MW_DEBUG_LOG
    RCLCPP_INFO(SLROSNodePtr->get_logger(),"** Base rate task semaphore received\n");
#endif
    if (!mRunModel) break;
	/* External mode */
    extmodeSimulationTime_T currentTime = (extmodeSimulationTime_T) (sim4_ROS2_delta_M)->Timing.taskTime0;
    {
        boolean_T rtmStopReq = false;
        rtmStopReq = !((rtmGetErrorStatus(sim4_ROS2_delta_M) == (NULL)));
        mRunModel = !rtmStopReq && !extmodeSimulationComplete() && !extmodeStopRequested();
        if (mRunModel == false) {
            rtmSetErrorStatus(sim4_ROS2_delta_M, "Simulation finished");
            break;
        }
    }
    sim4_ROS2_delta_step(
	);
    extmodeEvent(0, currentTime);
    mRunModel = !NodeInterface::getStopRequestedFlag();
  }
  NodeInterface::stop();
}
void NodeInterface::extmodeBackgroundTask(void)
{
  while (mRunModel) {
    /* External mode */
    extmodeBackgroundRun();
    // Sleep for 10 ms to yield to other threads
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
}//namespace matlab
}//namespace ros2
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER

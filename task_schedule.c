#include <stdio.h>
#include <stdlib.h>

#include "task_schedule.h"
#include "task_queue.h"
#include "FreeRTOS.h"


#ifndef TASK_SCHELDULE_H
#define TASK_SCHEDULE_H

static void idleTaskPrint(void) {
    printf("Idle task run \r\n");
}

static task_infomation_t idle_task_info = {1, MAX_TASK_NUMBER + 1, 1, 0, 0, idleTaskPrint};
static task_t idle_task = {&idle_task_info, 0, 0, 0, task_ready};

static inline task_t* getTask() {
    int i = 0;
    task_t * foundTask = NULL;
    for (; i < MAX_TASK_NUMBER; i++) {
        if ((gTaskQueue[i].taskStatus == task_ready) ||
            (gTaskQueue[i].taskStatus == task_running)) {
            
            if (foundTask == NULL) {
                foundTask = &gTaskQueue[i];
            } else if (gTaskQueue[i].taskStatus == task_running) {
                gTaskQueue[i].taskStatus = task_suspend;
            } /* end inner if */

        } /* end if */
    } /* end for */

    if (foundTask != NULL) {
        return foundTask;
    } else {
        return &idle_task;
    } /* end if */
}

static schedule_type_t schedule_select = task_schedule_unknow;

/**
 * \brief re-arrange task base on select schedule type 
 */
static bool arrangeTaskPosition() {
    bool retVal = 0;
    switch (schedule_select)
    {
    case task_schedule_priority:
        reArrangeQueueByPriority();
        break;
    case task_schedule_EDF:
        reArrangeQueueByEDF();
        break;
    case task_schedule_LLS:
        reArrangeQueueByLLS();
        break;
    case task_schedule_custom:
        reArrangeQueueCustomSchedule();
        break;
    case task_schedule_unknow:
        retVal = 1;
        break;
    default:
        /* should not reach here */
        retVal = 1;
        break;
    }

    return retVal;
}

/**
 * \brief setup scheduler before start schedule, initial queue
 * arrange queue for first time run 
 */
void prepareSchedule() {
    initialTaskQueue();

    if (arrangeTaskPosition() != 0) {
        printf("Unknow schedule type or unknow status appear \r\n");
    }
}

/**
 * \brief set schedule type for scheduler 
 */
void setSchedulerType(schedule_type_t type) {
    schedule_select = type;
}

/**
 * \brief schedule task for all task in queue
 */
bool startSchedule() {
    task_t* selectedTask = NULL;
    //while (true) {
        selectedTask = getTask();
        selectedTask->taskStatus = task_running;
        if(selectedTask->cycleNum>=1){
            selectedTask = &idle_task;
            return 1;
        }
        selectedTask->task_info->taskAction(NULL);
        /* delay for 1 sec */
        vTaskDelay(1000);
        /* task queue variable update */
        updateTimeForAllTask(1);
        updateStatusForAllTask();
        /* re-arange queue */
        arrangeTaskPosition();
    //} /* end while */
		
		
    /*while(true){
        selectedTask->taskStatus = task_running;
        selectedTask->task_info->taskAction(NULL);
        vTaskDelay(1000);
    }*/
		return 0;
}


#endif /* TASK_SCHEDULE_H */
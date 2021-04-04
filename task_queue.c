#include <stdio.h>

#include "task_queue.h"
#include "coffee_task.h"
#include "servo.h"

task_t gTaskQueue[MAX_TASK_NUMBER] = {0}; /*!< global task queue */
static void task1_action(void* param) {
    printf("Task 1 is running \r\n");
		setServoAngle(2500);
}

static void task2_action(void* param) {
    printf("Task 2 is running \r\n");
		setServoAngle(2000);
}

static void task3_action(void* param) {
    printf("Task 3 is running \r\n");
		setServoAngle(1500);
}

static void task4_action(void* param) {
    printf("Task 4 is running \r\n");
		setServoAngle(1000);
}

static const task_infomation_t taskList[MAX_TASK_NUMBER] = 
{
    {2, 1, 10, 3, 5, &task1_action},
    {3, 2, 20, 1, 7, &task2_action},
    {3, 3, 20, 2, 7, &task3_action},
    {4, 4, 30, 2, 10, &task4_action}
};

/**
 * \brief re-arrange task queue base on priority of each task 
 */
void reArrangeQueueByPriority() {
    /* use insertion sort */
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        for (int j = 0; j < i; j++) {
            if (compareTaskByPriority(&gTaskQueue[i], &gTaskQueue[j]) == 1) {
                swapTask(&gTaskQueue[i], &gTaskQueue[j]);
            } /* end if */
        } /* end nested for */
    } /* end for */
}

/**
 * \brief re-arrange task queue base on EDF of task 
 */
void reArrangeQueueByEDF() {
    /* use insertion sort */
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        for (int j = 0; j < i; j++) {
            if (compareTaskEDF(&gTaskQueue[i], &gTaskQueue[j]) == 1) {
                swapTask(&gTaskQueue[i], &gTaskQueue[j]);
            } /* end if */
        } /* end nested for */
    } /* end for */
}

/** 
 * \brief re-arrange task queue base on LLS of task
 */
void reArrangeQueueByLLS() {
    /* use insertion sort */
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        for (int j = 0; j < i; j++) {
            if (compareTaskLLS(&gTaskQueue[i], &gTaskQueue[j]) == 1) {
                swapTask(&gTaskQueue[i], &gTaskQueue[j]);
            } /* end if */
        } /* end nested for */
    } /* end for */
}

/** 
 * \brief re-arrange task queue by custom task schedule 
 */
void reArrangeQueueCustomSchedule() {
    /* use insertion sort */
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        for (int j = 0; j < i; j++) {
            if (compareTaskSJF(&gTaskQueue[i], &gTaskQueue[j]) == 1) {
                swapTask(&gTaskQueue[i], &gTaskQueue[j]);
            } /* end if */
        } /* end nested for */
    } /* end for */

}

/**
 * \brief intial task queue for use
 */
void initialTaskQueue() {
    for (int i = 0; i < MAX_TASK_NUMBER; i ++) {
        gTaskQueue[i].task_info = &taskList[i];
        gTaskQueue[i].taskStatus = task_ready;
    }
}

/**
 * \brief dynamic reset task queue 
 */
void resetTaskQueue() {
    memset(gTaskQueue, 0, sizeof(task_t) * MAX_TASK_NUMBER);
    for (int i = 0; i < MAX_TASK_NUMBER; i ++) {
        gTaskQueue[i].task_info = &taskList[i];
        gTaskQueue[i].taskStatus = task_ready;
    } /* end for */
}

/**
 * \brief update time counter waiting for all task in queue
 */
void updateTimeForAllTask(int time_increase) {
    task_t *selectTask = NULL;
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        selectTask = &gTaskQueue[i];
        switch (selectTask->taskStatus)
        {
        case task_ready:
            selectTask->counterToDeadline += time_increase;
            selectTask->waittingTime += time_increase;
            break;
        case task_running:
            selectTask->currentTimeConsume += time_increase;
            selectTask->waittingTime += time_increase;
            break;
        case task_suspend:
            selectTask->waittingTime += time_increase;
            break;
        case task_done:
            selectTask->waittingTime += time_increase;
            break;
        default:
            break;
        } /* end switch*/
    } /* end for */
}

/**
 * \brief update status for all task in queue base on task attribute 
 */
void updateStatusForAllTask() {
    task_t *selectTask = NULL;
    for (int i = 0; i< MAX_TASK_NUMBER; i++) {
        selectTask = &gTaskQueue[i];
        switch (selectTask->taskStatus)
        {
        case task_running:
            if (selectTask->currentTimeConsume == selectTask->task_info->timeForWorking) {
                selectTask->taskStatus = task_done;
                if(selectTask->task_info->taskId==4){
                    selectTask->cycleNum++;
                }
                selectTask->currentTimeConsume = 0;
                selectTask->counterToDeadline = 0;
            } /* end if */
            break;
        case task_done:
            if (selectTask->waittingTime == selectTask->task_info->period) {
                selectTask->taskStatus = task_ready;
                /* when turn to ready, task will increase time to deadline 
                 * instead of waitting time so must set here for continue
                 * counter for next period */
                selectTask->waittingTime = 0; 
            }
            break;
        case task_suspend:
            /* only suspend when higher priority task come so 
             * set it to ready on next cpu circle */
            selectTask->taskStatus = task_ready;  
            if (selectTask->waittingTime == selectTask->task_info->period) {
                /* when task be suspended the period of task still same so 
                 * task will increase time waitting for next period, 
                 * must set here for continue counter for next period */
                selectTask->waittingTime = 0; 
            }
            break;
        case task_ready:
            /* will be handle and control by cpu*/
            if (selectTask->waittingTime == selectTask->task_info->period) {
                /* when task ready period of task still same, 
                 * period does not depend on task status 
                 * so task will increase time waitting for next period,
                 * must set here for continue counter for next period */
                selectTask->waittingTime = 0; 
            }
            break;
        
        default:
            break;
        } /* end switch */
    } /* end for loop */
}
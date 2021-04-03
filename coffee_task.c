#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "coffee_task.h"

/**
 * \brief check task is ready to run or not 
 */
static bool __isReadyToRun(const task_t* task) {
    if (task->taskStatus == task_running || task->taskStatus == task_ready) {
        return true;
    } else {
        return false;
    } /* end if */
}
/**
 * \brief Boolean function to check task is done or no 
 */
bool IsDone(task_t task) {
    if (task.currentTimeConsume == task.task_info->timeForWorking) {
        return true;
    } else if (task.currentTimeConsume < task.task_info->timeForWorking) {
        return false;
    } else {
        /* somethings wrong here, currrent consume time is bigger than time needed to complete */
        printf("TASK: Error occur when check task is done \r\n");
    }  /* end if */
    return false;
} /* end function */

/**
 * \brief Swap to task 
 */
void swapTask(task_t *first_task, task_t *second_task) {
    task_t temp_task = {0};
    memcpy(&temp_task, first_task, sizeof(task_t));     /* copy first task to temp */
    memcpy(first_task, second_task, sizeof(task_t));
    memcpy(second_task, &temp_task, sizeof(task_t));
} /* End function */

/**
 * \brief Compare task base on Priority 
 */
int compareTaskByPriority(const task_t *first_task, const task_t *second_task) 
{
    if (first_task->task_info->priority > second_task->task_info->priority) {
        return 1;
    } /* end if */

    if (first_task->task_info->priority < second_task->task_info->priority) {
        return -1;
    } /* end if */

    return 0;
} 

/**
 * \brief Compare task base on time to deadline 
 */
int compareTaskLLS(const task_t *first_task, const task_t *second_task) {
    if (__isReadyToRun(first_task) && __isReadyToRun(second_task)) {
        /* compute how long deadline will happend, negative number is accept
        * if task miss the deadline */
        int deadlineFirstTask = (first_task->task_info->deadline - first_task->counterToDeadline);
        int deadlineSecondTask = (second_task->task_info->deadline - second_task->counterToDeadline);

        if (deadlineFirstTask < deadlineSecondTask) {
            return 1;
        } /* end if */

        if (deadlineFirstTask > deadlineSecondTask) {
            return -1;
        } /* end if */

        return 0;
    } else if (__isReadyToRun(first_task)) {
        /* only first task is ready so set it to higher position to be select */
        return 1;
    } else if (__isReadyToRun(second_task)) {
        /* only second task is ready so set it to higher position to be select */
        return -1;
    } 
    return 0;
}

/**
 * \brief Compare task base on deadlline 
 */
int compareTaskEDF(const task_t *first_task, const task_t *second_task) {
    if (__isReadyToRun(first_task) &&  __isReadyToRun(second_task))
    {
        if (first_task->task_info->deadline < second_task->task_info->deadline) {
            return 1;
        } /* end if */

        if (first_task->task_info->deadline > second_task->task_info->deadline) {
            return -1;
        } /* end if */
    } else if (__isReadyToRun(first_task)) {
        /* only first task is ready so set it to higher position to be select */
        return 1;
    } else if (__isReadyToRun(second_task)) {
        /* only second task is ready so set it to higher position to be select */
        return -1;
    }

    return 0;
}

/**
 * \brief Compare task base on task execute time 
 */
int compareTaskSJF(const task_t *first_task, const task_t *second_task) {
    if (__isReadyToRun(first_task) &&  __isReadyToRun(second_task))
    {
        int firstTaskCpuTimeRemain = first_task->task_info->timeForWorking - first_task->currentTimeConsume;
        int secondTaskCpuTimeRemain = second_task->task_info->timeForWorking - second_task->currentTimeConsume;

        if (firstTaskCpuTimeRemain < secondTaskCpuTimeRemain) {
            return 1;
        } /* end if */

        /* current running task will have higher position, so if equal happen
         * refer current runing task to continue running for not switch context */
        if (firstTaskCpuTimeRemain >= secondTaskCpuTimeRemain) {
            return -1;
        } /* end if */
    } else if (__isReadyToRun(first_task)) {
        /* only first task is ready so set it to higher position to be select */
        return 1;
    } else if (__isReadyToRun(second_task)) {
        /* only second task is ready so set it to higher position to be select */
        return -1;
    }

    return 0;
}
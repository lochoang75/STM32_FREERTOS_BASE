#include "coffee_task.h"

#ifndef TASK_QUEUE_H
#define TASK_QUEUE_H

#define MAX_TASK_NUMBER   4

extern task_t gTaskQueue[MAX_TASK_NUMBER];

void initialTaskQueue();

void resetTaskQueue();

void reArrangeQueueByPriority();

void reArrangeQueueByEDF();

void reArrangeQueueByLLS();

void reArrangeQueueCustomSchedule();

void updateTimeForAllTask(int time_increase);

void updateStatusForAllTask();
 

#endif /*TASK_QUEUE_H */
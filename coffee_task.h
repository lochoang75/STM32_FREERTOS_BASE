#include <stdbool.h>

#ifndef COFFEE_TASK_H
#define COFFEE_TASK_H
/**
 * \brief Function pointer point to handle task
 */
typedef void (*pAction_t)(void* param);

typedef enum task_status {
    task_ready = 0,             /*!< Task is ready to run, only need cpu to run */
    task_running,               /*!< Task is using CPU */
    task_done,                  /*!< Task is just finish one circle */
    task_suspend,               /*!< Task is suspend for preemptive by another task */
    task_block,                 /*!< Task is block after schedule is finish */
} task_status_t;

typedef struct task_infomation {
    unsigned int timeForWorking;                /*!< Time use cpu when task running */
    unsigned int taskId;                        /*!< Task ID to know which task is running */
    unsigned int period;                        /*!< Total CPU time to complete task */
    unsigned int priority;                      /*!< Task prioprity */
    unsigned int deadline;                      /*!< Task deadline for waitting to run */
    const pAction_t taskAction;                 /*!< Function pointer for task woking */
} task_infomation_t;
/**
 * \brief Task definition abstract struct
 */
typedef struct task {
    const task_infomation_t *task_info;               /*!< Constant task information on initial */
    unsigned int currentTimeConsume;                  /*!< Current time consume for this task */
    unsigned int waittingTime;                        /*!< Current waitting time from last run */
    unsigned int counterToDeadline;                   /*!< Time to deadline when task start new circle */ 
    unsigned int cycleNum;                             /*!< Indicate the number of cycles finished */
    task_status_t taskStatus;                         /*!< Task status */
} task_t;

bool IsDone(task_t task);

void swapTask(task_t *first_task, task_t *second_task);

int compareTaskByPriority(const task_t *first_task, const task_t *second_task);

int compareTaskEDF(const task_t *first_task, const task_t *second_task);

int compareTaskLLS(const task_t *first_task, const task_t *second_task);

int compareTaskSJF(const task_t *frist_task, const task_t *second_task);
#endif
#ifndef TASK_SCHEDULE_H
#define TASK_SCHEDULE_H
#include "stdbool.h"
/**
 * \enum schedule_type_t
 * \brief schedule type for queue, base on this value task will be sort and run
 */
typedef enum shdule_type {
    task_schedule_unknow = 0,                   /*!< Unset schedule type for system */
    task_schedule_priority,                     /*!< Set schedule type to base on priority */
    task_schedule_EDF,                          /*!< Set schedule type to EDF */
    task_schedule_LLS,                          /*!< Set schedule type to LLS */
    task_schedule_custom,                       /*!< Set schedule type to custom */
} schedule_type_t;

bool startSchedule();

void prepareSchedule();

void setSchedulerType(schedule_type_t type);

#endif
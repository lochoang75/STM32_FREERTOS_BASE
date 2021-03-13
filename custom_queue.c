#include "FreeRTOS/Source/include/semphr.h"
#include "FreeRTOS/Source/portable/RVDS/ARM_CM4F/portmacro.h"

#include "custom_queue.h"
#define MAX_QUEUE_SIZE  20

static select_tone_t toneArr[MAX_QUEUE_SIZE];
static tone_queue_t gToneQueue = {
    .headPtr = NULL,
    .size = 0,
    .head = 0,
    .tail = 0,
};

static inline void __copyTone(select_tone_t *dst, select_tone_t* src) {
    dst->duration = src->duration;
    dst->tone = src->tone;
}

static inline void __queueGetKey(SemaphoreHandle_t *key) {
    xSemaphoreTake(key, portMAX_DELAY);
}

static inline void __queueReturnKey(SemaphoreHandle_t *key) {
    xSemaphoreGive(key);
}

queue_error_t create_queue (unsigned int size) {
    if (size > MAX_QUEUE_SIZE) {
        // ERROR: cannot allocate queue size bigger than max size
        return  qerr_not_initialized;
    } // end if

    vSemaphoreCreateBinary(&gToneQueue.key);
    if (gToneQueue.key != NULL) {
        gToneQueue.headPtr = &toneArr[0]; // point to head of array
        gToneQueue.size = size;
        return qerr_success;
    } // end if
}

tone_queue_t * get_instance() {
    return &gToneQueue;
}

queue_error_t push_back(tone_queue_t *queue, select_tone_t tone_select) {
    queue_error_t error = qerr_success;
    __queueGetKey(&queue->key);
    if (isInitialize(queue) && !isFull(queue)) {
        __copyTone(&queue->headPtr[queue->head], &tone_select);
        queue->head ++; 
    } else {
        if (!isInitialize(queue)) {
            error = qerr_not_initialized;
        } else {
            error = qerr_full;
        } // end nested if
    } // end if

    __queueReturnKey(&queue->key);

    return error;
}

queue_error_t get_tone(tone_queue_t *queue, select_tone_t *output) {
    queue_error_t error = qerr_success;
    __queueGetKey(&queue->key);
    if (isInitialize(queue) && !isEmpty(queue)) {
        __copyTone(output, &queue->headPtr[queue->tail]);
        queue->tail ++; 

        // do not let tail go before head, this is a single way queue
        if (queue->tail > queue->head) {
            queue->tail == 0;
        } // end nested looop
    } else {
        if (!isInitialize(queue)) {
            error = qerr_not_initialized;
        } else {
            error = qerr_empty;
        } // end nested if
    } // end if
    __queueReturnKey(&queue->key);

    return error;
}

queue_error_t traverse(tone_queue_t *queue, void (*callback)(select_tone_t*)) {
    queue_error_t error = qerr_success;
    __queueGetKey(&queue->key);
    if (isInitialize(queue) && !isEmpty(queue)) {
        for (int i = 0; i < queue->head; i++) {
            (*callback)(&queue->headPtr[i]);
        }
    } else {
        if (!isInitialize(queue)) {
            error = qerr_not_initialized;
        } else {
            error = qerr_full;
        } // end nested if
    } // end if
    __queueReturnKey(&queue->key);

    return error;
}

queue_error_t clean_queue(tone_queue_t *queue) {
    queue_error_t error = qerr_success;
    __queueGetKey(&queue->key);
    if (isInitialize(queue)) {
        queue->head = 0;
        queue->tail = 0;
    } else {
        error = qerr_not_initialized;
    } // end if 
    __queueReturnKey(&queue->key);

    return error;
}

queue_error_t destroy_queue(tone_queue_t *queue) {
    queue_error_t error = qerr_success;
    vSemaphoreDelete(&queue->key);
    queue->headPtr = NULL;
    queue->size = 0;
    queue->head = 0;
    queue->tail = 0;
}
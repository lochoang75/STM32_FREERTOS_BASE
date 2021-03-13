#ifndef CUSTOM_QUEUE_H
#define CUSTOM_QUEUE_H
#include "semphr.h"

#pragma pack(1)
/*!
    \brief custom type definition for tone
 */
typedef struct select_tone {
    unsigned int tone;
    unsigned int duration; // in ms
} select_tone_t;

typedef struct tone_queue {
    select_tone_t *headPtr;
    SemaphoreHandle_t key; // protect the queue
    unsigned int size; // queue size
    unsigned int head;
    unsigned int tail;
} tone_queue_t;

typedef enum queue_error {
    qerr_success = 0,
    qerr_full,
    qerr_empty,
    qerr_busy,
    qerr_not_initialized,
    qerr_unknow,
} queue_error_t;

inline bool isFull(tone_queue_t *queue) {
    return (queue->head == queue->size) ? true : false;
}

inline bool isEmpty(tone_queue_t *queue) {
    return (queue->head == 0) ? true: false;
}

inline bool isInitialize(tone_queue_t *queue) {
    return (queue->size == 0) ? true: false; 
}

/*!
    \brief create the tone queue with specific size
    \param size size of the queue
 */
queue_error_t create_queue(unsigned int size);

/*!
    \brief get the pointer  reference to the global queue address 
  */
tone_queue_t* get_instance();

/*!
    \brief push selected tone to back of the queue
    \param queue selected queue to performed push
    \param tone_select selected tone to push to queue
    \return error when push 
 */
queue_error_t push_back(tone_queue_t* queue, select_tone_t tone_select); 

/*!
    \brief get tone from the tail of the 
    \param queue selected queue to get
    \param ouput output tone get from queue
    \return error code
 */
queue_error_t get_tone(tone_queue_t* queue, select_tone_t *output);

/*!
    \brief traverse from begin to head and peform callback all any elements
    \param queue selected queue to traverse
    \param callback action needed on each elements
 */
queue_error_t traverse(tone_queue_t* queue, void (*callback)(select_tone_t*));

/*! 
    \brief discard all data that queue contains, will reset the queue as the initial value
 */
queue_error_t clean_queue(tone_queue_t *queue);

/*!
    \brief destroy the queue
*/
queue_error_t destroy_queue(tone_queue_t *queue);
#endif
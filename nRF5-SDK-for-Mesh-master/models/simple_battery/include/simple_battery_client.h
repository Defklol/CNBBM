#ifndef SIMPLE_BATTERY_CLIENT_H__
#define SIMPLE_BATTERY_CLIENT_H__

#include <stdint.h>
#include "access.h"
#include "simple_battery_common.h"


/** Simple OnOff Client model ID. */
#define SIMPLE_BATTERY_CLIENT_MODEL_ID (0x0001)

/** Simple OnOff status codes. */
typedef enum
{
    /** Received status from the server. */
    SIMPLE_BATTERY_STATUS_RECEIVE,
    /** The server did not reply to a Simple OnOff Set/Get. */
    SIMPLE_BATTERY_STATUS_ERROR_NO_REPLY,
    /** Simple OnOff Set/Get was cancelled. */
    SIMPLE_BATTERY_STATUS_CANCELLED
} simple_battery_status_t;

/** Forward declaration. */
typedef struct __simple_battery_client simple_battery_client_t;

/**
 * Simple OnOff status callback type.
 *
 * @param[in] p_self Pointer to the Simple OnOff client structure that received the status.
 * @param[in] status The received status of the remote server.
 * @param[in] src    Element address of the remote server.
 */
typedef void (*simple_battery_status_cb_t)(const simple_battery_client_t * p_self, simple_battery_status_t status, uint16_t src);

//typedef void (*simple_battery_set_unreliable_cb_t)(const simple_battery_client_t *p_self, uint8_t bat_value, uint16_t src);
/**
 * Simple OnOff timeout callback type.
 *
 * @param[in] handle Model handle
 * @param[in] p_self Pointer to the Simple OnOff client structure that received the status.
 */
typedef void (*simple_battery_timeout_cb_t)(access_model_handle_t handle, void * p_self);

/** Simple OnOff Client state structure. */
struct __simple_battery_client
{
    /** Model handle assigned to the client. */
    access_model_handle_t model_handle;
    /** Status callback called after status received from server. */
    //simple_battery_status_cb_t status_cb;

    //simple_battery_set_unreliable_cb_t set_unreliable_cb;
    /** Timeout callback called after acknowledged message sending times out */
    simple_battery_timeout_cb_t timeout_cb;
    /** Internal client state. */
    struct
    {
        bool reliable_transfer_active; /**< Variable used to determine if a transfer is currently active. */
        simple_battery_msg_set_t data;  /**< Variable reflecting the data stored in the server. */
    } state;

    
};

/**
 * Initializes the Simple OnOff client.
 *
 * @note This function should only be called _once_.
 * @note The client handles the model allocation and adding.
 *
 * @param[in,out] p_client      Simple OnOff Client structure pointer.
 * @param[in]     element_index Element index to add the server model.
 *
 * @retval NRF_SUCCESS         Successfully added client.
 * @retval NRF_ERROR_NULL      NULL pointer supplied to function.
 * @retval NRF_ERROR_NO_MEM    No more memory available to allocate model.
 * @retval NRF_ERROR_FORBIDDEN Multiple model instances per element is not allowed.
 * @retval NRF_ERROR_NOT_FOUND Invalid element index.
 */
uint32_t simple_battery_client_init(simple_battery_client_t * p_client, uint16_t element_index);

/**
 * Sets the state of the Simple OnOff server.
 *
 * @param[in,out] p_client Simple OnOff Client structure pointer.
 * @param[in]     battery   Value to set the Simple OnOff Server state to.
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */
uint32_t simple_battery_client_set(simple_battery_client_t * p_client, uint8_t value);

/**
 * Sets the state of the Simple OnOff Server unreliably (without acknowledgment).
 *
 * @param[in,out] p_client Simple OnOff Client structure pointer.
 * @param[in]     on_off   Value to set the Simple OnOff Server state to.
 * @param[in]     repeats  Number of messages to send in a single burst. Increasing the number may
 *                     increase probability of successful delivery.
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */
uint32_t simple_battery_client_set_unreliable(simple_battery_client_t * p_client, uint8_t value, uint8_t repeats, uint16_t src);

/**
 * Gets the state of the Simple OnOff server.
 *
 * @note The state of the server will be given in the @ref simple_on_off_status_cb_t callback.
 *
 * @param[in,out] p_client Simple OnOff Client structure pointer.
 *
 * @retval NRF_SUCCESS              Successfully sent message.
 * @retval NRF_ERROR_NULL           NULL pointer in function arguments
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for a reliable transfer.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 */
uint32_t simple_battery_client_get(simple_battery_client_t * p_client);

/**
 * Cancel any ongoing reliable message transfer.
 *
 * @param[in,out] p_client Pointer to the client instance structure.
 */
void simple_battery_client_pending_msg_cancel(simple_battery_client_t * p_client);



/** @} end of SIMPLE_ON_OFF_CLIENT */

#endif /* SIMPLE_ON_OFF_CLIENT_H__ */

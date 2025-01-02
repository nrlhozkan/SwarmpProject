import requests

from utils import config
from utils.logging_utils import log, LOG_TYPE_INFO, LOG_TYPE_WARNING, LOG_TYPE_SUCCESS


def check_response(operation_type, response):
    if response.status_code == 200:
        log(LOG_TYPE_SUCCESS, f'Operation {operation_type} executed successfully', response.text)
    else:
        log(LOG_TYPE_SUCCESS, f'Failed to execute operation {operation_type}',
            [str(response.status_code), response.text])


def delete_user_data(session):
    # Delete user data while keeping settings
    resp = session.post(config.DELETE_DATA_KEEP_SETTINGS_URL)
    check_response('Delete user data', resp)


def send_message(session, message, c_id, debug) -> None:
    # log(LOG_TYPE_INFO, f'Prompting message', [message, f'chat ID: {c_id}'])
    log(LOG_TYPE_INFO, f'Prompting message to Charlie', [f'chat ID: {c_id}'])

    if not debug:
        message_data = {
            'prompt': message,
            'display_name': config.DISPLAY_NAME,
            'chat_id': c_id,
            'username': 'admin',
        }

        # prompt to Charlie and read response
        charlie_resp = session.post(config.MESSAGE_URL, json=message_data)
        check_response('Send message', charlie_resp)
        
        return charlie_resp.text
    
    else:
        log(LOG_TYPE_WARNING, 'Debug mode active - no message sent')
        
        return None


def set_up():
    # Create a session to persist cookies across requests
    new_session = requests.Session()

    # Login request
    response = new_session.post(config.LOGIN_URL, json=config.LOGIN_DATA)

    # Check if login was successful
    check_response('Login', response)

    if response.status_code == 200:
        # Extract the session token and username from the response cookies
        # Set the session token and username cookies in the session object
        new_session.cookies.set('session_token', response.cookies.get('session_token'))
        new_session.cookies.set('username', response.cookies.get('username'))
    else:
        exit(1)

    return new_session


def set_up_charlie(chat_json, c_id, reset_data=False):
    # set up
    curr_session = set_up()

    # delete user data & chats
    if reset_data:
        delete_user_data(curr_session)

    # create new chat
    response = curr_session.post(config.CREATE_CHAT_TAB_URL, json=chat_json)
    check_response('Create new chat', response)

    # set newly created tab as active
    response = curr_session.post(
        config.SET_ACTIVE_TAB_URL, json={'username': 'admin', 'chat_id': c_id}
    )
    check_response('Set chat active', response)

    return curr_session

# CHARLIE CONFIGURATION

LOGIN_URL = 'http://localhost:8002/login/'
MESSAGE_URL = 'http://localhost:8002/message/'
CREATE_CHAT_TAB_URL = 'http://localhost:8002/create_chat_tab/'
SET_ACTIVE_TAB_URL = 'http://localhost:8002/set_active_tab/'
DELETE_DATA_KEEP_SETTINGS_URL = 'http://localhost:8002/delete_data_keep_settings/'
DELETE_DATA_URL = 'http://localhost:8002/delete_data/'
LOGIN_DATA = {'username': 'admin', 'password': 'admin'}
DISPLAY_NAME = 'Admin'

# DATA STORAGE CONFIGURATION

# this file is read by charlie to place the drones
INITIAL_DRONE_POS_FILE = r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data\initial_drone_pos.json' 

# this is where charlie stores the positions (of one step)
DRONE_POS_FILE = r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data\drone_pos.json'  

# files and directories for collected information about experiments
EXPERIMENT_BASE_FOLDER = r'C:\Users\cgadmin\Desktop\charlie\experiments'
# TODO this is where the experiment data will be stored
#   for each time the script is executed all the experiment data will be stored in a new directory (see EXPERIMENT_FOLDER_FORMAT below)

EXPERIMENT_FOLDER_FORMAT = 'data_%d_%m_%Y_%H_%M_%S'

# stored in the experiment folder
ALL_DRONE_POS_FILE = 'drone_positions.json'
ALL_TARGET_POS_FILE = 'target_positions.json'
ALL_DISTANCES_FILE = 'drone_target_distances.json'

NUM_COLORS = 10

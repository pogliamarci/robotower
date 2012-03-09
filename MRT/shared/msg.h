#ifndef MSG_H
#define MSG_H

#include <string>

/* definizione dei tipi di messaggio
   che si possono scambiare i nostri robot */

const int MSG_FROM_VISION = 18;
const int MSG_FROM_VISION_OMNIDIR = 19;
const int MSG_FROM_VISION_FRONTAL = 20;
const int MSG_FROM_MOTION = 21;
const int MSG_FROM_MONITOR = 22;
const int MSG_FROM_MAP = 23;
const int MSG_FROM_SCARE = 24;
const int MSG_FROM_BRIAN = 25;
const int MSG_FROM_BIG_BROTHER = 26;
const int MSG_FROM_ANCHORER = 27;
const int MSG_FROM_REFBOX = 28;

const int MSG_TO_BRIAN  = 31;
const int MSG_TO_MOTION = 32;
const int MSG_TO_BIG_BROTHER = 33;
const int MSG_TO_ANCHORER = 34;

const int MSG_TO_SERVO = 80;
const int MSG_TO_SERVO_AUDIO = 81;


const std::string COMMAND_DATA = "command_data";
const std::string BRIAN_DATA = "brian_data";
const std::string LOCAL_MAP_DATA = "local_map_data";
const std::string MAP_DATA = "map_data";
const std::string MONITOR_DATA = "monitor_data";
const std::string VISION_DATA = "vision_data";
const std::string VISION_DATA_OUTER = "vision_data_outer";
const std::string VISION_DATA_INNER = "vision_data_inner";
const std::string VISION_DATA_OMNIDIR = "vision_data_omnidir";
const std::string VISION_DATA_FRONTAL = "vision_data_frontal";
const std::string MOTION_DATA = "motion_data";
const std::string ANCHORING_DATA = "anchoring_data";
const std::string REFBOX_DATA = "refbox_data";

const std::string RED_BLOB_DATA = "RedBlob";
const std::string BLACK_BLOB_DATA = "BlackBlob";
const std::string YELLOW_BLOB_DATA = "YellowBlob";
const std::string BLUE_BLOB_DATA = "BlueBlob";
const std::string CYAN_BLOB_DATA = "CyanBlob";
const std::string MAGENTA_BLOB_DATA = "MagentaBlob";

const std::string BLOB_MIN_DISTANCE_DATA = "MinDistance";
const std::string BLOB_MAX_DISTANCE_DATA = "MaxDistance";
const std::string BLOB_MIN_ANGLE_DATA = "MinAngle";
const std::string BLOB_MAX_ANGLE_DATA = "MaxAngle";
const std::string BLOB_HEIGHT_DATA = "Height";

const std::string BLUE_GOAL_PERCEPTION_DATA = "Blue";
const std::string YELLOW_GOAL_PERCEPTION_DATA = "Yellow";
const std::string YELLOW_BLUE_TRANSITION_DATA = "YB";
const std::string BLUE_YELLOW_TRANSITION_DATA = "BY";
const std::string BLUE_YELLOW_BLUE_TRANSITION_DATA = "BYB";
const std::string YELLOW_BLUE_YELLOW_TRANSITION_DATA = "YBY";

const std::string WHITE_PERCEPTION_DATA = "W";
const std::string GREEN_WHITE_TRANSITION_DATA = "GW";
const std::string WHITE_GREEN_TRANSITION_DATA = "WG";
const std::string GREEN_WHITE_GREEN_TRANSITION_DATA = "GWG";
const std::string CORNER_DATA = "Corner";
const std::string LINE_DATA = "Line";

#ifdef FACE_DETECTOR
	const std::string VISION_FACE_DATA = "FaceData";
	const std::string VISION_FACE_POS_X = "FacePosX";
	const std::string VISION_FACE_POS_Y = "FacePosY";
#endif


const std::string LINE_NLX = "lineNLX";
const std::string LINE_NRX = "lineNRX";
const std::string LINE_ELX = "lineELX";
const std::string LINE_ERX = "lineERX";
const std::string LINE_SLX = "lineSLX";
const std::string LINE_SRX = "lineSRX";
const std::string LINE_WLX = "lineWLX";
const std::string LINE_WRX = "lineWRX";
const std::string BUMP_NRX = "bumpNRX";
const std::string BUMP_NLX = "bumpNLX";
const std::string BUMP_SRX = "bumpSRX";
const std::string BUMP_SLX = "bumpSLX";


const int MSG_FROM_JOYPAD_BRIEF		= 43;
const std::string JOYPAD_DATA_BRIEF	= "joypad_data_brief";
const std::string PAD_V 		= "pad_v";
const std::string PAD_W 		= "pad_w";
const std::string OP_MODE 		= "op_mode";



const int MSG_FROM_SONAR_QUALITY	= 44;
const std::string SONAR_QUALITY		= "sonar_quality";
const std::string SONAR_QUALITY_DATA	= "sonar_quality_data";


const int MSG_FROM_VISION_TO_BRIAN = 46;

/*
const std::string LINE_NRX              = "lineNRX";
const std::string LINE_NLX              = "lineNLX";
const std::string LINE_SRX              = "lineSRX";
const std::string LINE_SLX              = "lineSLX";
const std::string LINE_WRX              = "lineWRX";
const std::string LINE_WLX              = "lineWLX";
const std::string LINE_ERX              = "lineERX";
const std::string LINE_ELX              = "lineELX";



const std::string BUMP_NRX              = "bumpNRX";
const std::string BUMP_NLX              = "bumpNLX";
const std::string BUMP_SRX              = "bumpSRX";
const std::string BUMP_SLX              = "bumpSLX";
*/


const int MSG_AUDIO_REQUEST		= 45;
const std::string AUDIO_REQUEST	= "joypad_data_audio";


const std::string D_OPEN_DATUM			= "<D>";
const std::string D_CLOSE_DATUM			= "</D>";





const std::string LOOK_FOR_PERSON = "lookforperson";
const std::string INTERACT_WITH_PERSON = "interact";
const std::string FACE = "face";



#endif

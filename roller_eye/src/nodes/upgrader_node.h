#ifndef __upgrader_node__
#define __upgrader_node__

typedef enum{
  OTA_UPGRADE_CMD_START,        //user start upgrade, whether the firmware is force to install is based on the install mode of the firmware
  OTA_UPGRADE_CMD_START_IN_WL,  //server commands the device start upgrading
  OTA_UPGRADE_CMD_DOWNLOAD,     //user choose download firmware and check
  OTA_UPGRADE_CMD_INSTALL,      //user choose install new firmware
  OTA_UPGRADE_CMD_CANCEL,       //cancel ota upgrade
  OTA_UPGRADE_CMD_NONE          //not any cmd
}OtaUpgradeCmd;

#endif /* defined(__upgrader_node__) */

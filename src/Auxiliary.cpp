//
// Created by rbdstudent on 17/06/2021.
//

#include "Auxiliary.h"

std::string Auxiliary::GetGeneralSettingsPath()
{
    std::filesystem::path currentDirPath = std::filesystem::current_path();
    std::string settingPath = currentDirPath.string();
    settingPath += "/../generalSettings.json";
    return settingPath;
}

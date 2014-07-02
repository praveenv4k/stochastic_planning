/*! 
 *  \author    Shashank Pathak
 *  \date      2012
 */
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <fstream>
#include <json/json.h>
#include <iostream>

#define JINDEX (Json::Value::ArrayIndex)
#define CONFIGFOLDER "../config/"
#define MASTERCONFIG "master.json"
#define PATH2CONFIG "../config/configtable.json"

/**
 * @brief JSON Configuration class
 **/
class Config
{

public:
  /**
   * @brief Single instance of the Config class
   *
   * @return Config* - Pointer to the Config class
   **/
  static Config* instance();
  /**
   * @brief Refresh the config (re-read the config file)
   *
   * @return Config*
   **/
  static Config* refresh();
  /**
   * @brief Root element of the JSON file
   **/
  Json::Value root;
  /**
   * @brief Total time
   **/
  double totTime;
  bool hasCollided;

  /**
   * @brief Destructor
   *
   **/
  ~Config()
    {
        instanceFlag = false;
    }
    /**
     * @brief Version of the configuration file
     *
     * @return int - Returns the version numner
     **/
    static int GetVersion();
    /**
     * @brief Set the version number of the config file
     *
     * @param version Version number
     * @return void
     **/
    static void SetVersion(int version);
private:
    static bool instanceFlag;
    static bool isParsed;
    static int m_version;
    static Config *single;      // Trivia: This is thread-safe only for GNU compiler and also for c++11,
                                // we dont give damn for airhockey as of now since its serial!
    Config()
    {
        
	std::string configFolder(CONFIGFOLDER);
	std::string masterConfigFile(MASTERCONFIG);
	std::string masterFilePath = configFolder+masterConfigFile;
	std::string actualConfigFile;
	std::cout << "Master config: " << masterFilePath << std::endl;
	{
	  Json::Reader rd;
	  Json::Value masterRoot;
	  std::ifstream tst(masterFilePath.c_str(), std::ifstream::binary);
	  bool parsed = rd.parse(tst,masterRoot,false);
	  if(parsed){
	    std::cout << "Master config parsed! " << std::endl;
	    actualConfigFile = masterRoot["config"].asString();
	    if(!actualConfigFile.empty()){
	      std::string actualFilePath = configFolder+actualConfigFile;
	      std::cout << "Actual config: " << actualFilePath << std::endl;
	      Json::Reader reader;
	      std::ifstream config(actualFilePath.c_str(), std::ifstream::binary);
	      isParsed = reader.parse(config,root,false);
	    }
	  }
	}
	m_version = 0;
        hasCollided = false;
    }
};

#endif
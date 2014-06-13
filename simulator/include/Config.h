#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <fstream>
#include <json/json.h>

#define JINDEX (Json::Value::ArrayIndex)
#define PATH2CONFIG "config.json"

class Config
{

public:
    static Config* instance();
    static Config* refresh();
    Json::Value root;
    double totTime;
    bool hasCollided;

    ~Config()
    {
        instanceFlag = false;
    }
    static int GetVersion();
    static void SetVersion(int version);
private:
    static bool instanceFlag;
    static bool isParsed;
    static int m_version;
    static Config *single;      // Trivia: This is thread-safe only for GNU compiler and also for c++11,
                                // we dont give damn for airhockey as of now since its serial!
    Config()
    {
        Json::Reader rd;
	std::ifstream tst(PATH2CONFIG, std::ifstream::binary);
	//std::ifstream tst("config.json", std::ifstream::binary);
        isParsed = rd.parse(tst,root,false);
        m_version = 0;
        hasCollided = false;
    }
};

#endif
#ifndef STORAGE_H
#define STORAGE_H

#include <FS.h>

using namespace std;

class Storage{
    public:
        virtual int makeDirectory(const char *path) = 0;
        virtual int removeDirector(const char *path) = 0;
        virtual int listDirectory(const char *path, uint8_t levels) = 0;

        virtual String readFile(const char *path) = 0;
        virtual int writeFile(const char *path, const char *data) = 0;
        virtual int appendFile(const char *path, const char *data) = 0;
        virtual int deleteFile(const char *path) = 0;
        virtual int renameFile(const char *pathA, const char *pathB) = 0;

    private:
        fs::FS fs;
};

#endif
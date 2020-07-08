/****************************************************************************\
*      --- Practical Course: GPU Programming in Computer Vision ---
*
* time:    winter term 2012/13 / March 11-18, 2013
*
* project: superresolution
* file:    filesystem.cpp
*
*
*             !THIS FILE IS SUPPOSED TO REMAIN UNCHANGED!
\****************************************************************************/

/*
 * filesystem.cpp
 *
 *  Created on: Feb 29, 2012
 *      Author: steinbrf
 */

#include "stdafx.h"
#include <stdio.h>
#include <filesystem.h>

Folder::Folder()
{

}

Folder::Folder(std::string p_path)
: path(p_path)
{
	load(p_path);
}

Folder::~Folder()
{}

bool Folder::load(std::string p_path)
{
  DIR *directory;
  struct dirent *file;
  directory = opendir(p_path.c_str());
  if(!directory) return false;
  while ((file = readdir(directory))){
    files.push_back(file->d_name);
  }
  closedir(directory);
  return true;
}

bool Folder::is_valid()
{
  DIR *directory;
  directory = opendir(path.c_str());
  if(!directory) return false;
  closedir(directory);
  return true;
}

size_t Folder::size()
{
	return files.size();
}

std::list<std::string> Folder::getFilesWithEndingList(std::string ending)
{
	std::list<std::string> result;
	for(unsigned int i=0;i<files.size();i++){
		if(files[i].size() >= ending.size()){
			if(files[i].substr(files[i].size()-ending.size(),ending.size()) == ending){
				result.push_back(files[i]);
			}
		}
	}
	result.sort();

	return result;
}

std::vector<std::string> Folder::getFilesWithEndingVector(std::string ending)
{
	std::list<std::string> namelist = getFilesWithEndingList(ending);

	return std::vector<std::string>(namelist.begin(),namelist.end());
}

bool Folder::fileExists(std::string filename,std::string prefix)
{
	for(unsigned int i=0;i<files.size();i++){
		if(prefix + files[i] == filename){
			return true;
		}
	}
	return false;
}

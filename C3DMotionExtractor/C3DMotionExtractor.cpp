
#include "stdafx.h"
#include "uuc3d.hpp"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <map>

#define _USE_MATH_DEFINES
#include <math.h>


using std::cerr;
using std::vector;
using UuIcsC3d::SpacePaddedString;
using UuIcsC3d::DataPoint3d;
using namespace boost::filesystem;

#define LABEL_START	6

vector<float> calcMotion(vector<float> point1, vector<float> point2, vector<float> point3)
{
	vector<float> motion(3);

	float center23X = (point2[0] + point3[0]) / 2;
	float center23Y = (point2[1] + point3[1]) / 2;
	float centerX = (point1[0] + center23X) / 2;
	float centerY = (point1[1] + center23Y) / 2;
	float deltaY = center23Y-centerY;
	float deltaX = center23X-centerX;
	float rot = atan2f(deltaY, deltaX);

	motion[0] = centerX/1000.0; // mm to m
	motion[1] = centerY/1000.0; // mm to m
	motion[2] = rot*(180/M_PI); // in degrees
	
	return motion;
}

void do_work(std::string filename, boost::property_tree::ptree pt)
{
	std::cout<< "Processing " << filename << std::endl;
    UuIcsC3d::C3dFileInfo fi(filename);
    int ppf = fi.points_per_frame(); 
    if (ppf <= 0)
	{
		return;
	}
    int fc = fi.frame_count();
	std::cout << "frames: " << fc << "\n";

	// retrieve the label in the filename and change the appropiate subject to active
	std::string fileLabel = filename.substr(filename.length()-LABEL_START, filename.length()); 
	fileLabel = fileLabel.substr(0,2);
	pt.put("subjects." + fileLabel +".role", "active");	

    // get labels and add dummy labels if necessary
    vector<SpacePaddedString> labels(fi.point_labels());
    for (int i=labels.size(); i<ppf; ++i) 
	{
		labels.push_back(SpacePaddedString("<no label>"));
    }

    // Open the C3D file
    std::auto_ptr<UuIcsC3d::C3dFile> filep(fi.open());

    // Read every frame in variable frame_data and process it
    UuIcsC3d::FrameData frame_data;

	// initialize some stuff
	// clear exisiting motion data
	for(auto keyval : pt.get_child("subjects")) 
	{
		pt.get_child("subjects." + keyval.first + ".motion").clear();
	}
	boost::property_tree::ptree m, mX, mY, mRot;
	std::map<std::string, std::pair<float,float>> points;
	vector<float> point1(2), point2(2), point3(2), motion(3);
	std::string label, label1, label2, label3;

	// loop through frames
    for (int i=0; i<fc; ++i)
	{
		if (i%500 == 0)
			std::cout<< "At frame: " << i << std::endl;
		filep->get_frame_data(frame_data, i);
		// loop through points
		for (int j=0; j<ppf; ++j) 
		{
			DataPoint3d const &dp=frame_data.points[j];
			if (labels[j].stripped().find("*") != 0) 
			{
				points[labels[j].stripped()].first = dp.x();
				points[labels[j].stripped()].second = dp.y();
			}
		}	
		// loop through subjects
		for(auto keyval : pt.get_child("subjects")) 
		{
			label = keyval.first;
			label1 = label + "1";
			label2 = label + "2";
			label3 = label + "3";

			point1[0] = points[label1].first;
			point1[1] = points[label1].second;
			point2[0] = points[label2].first;
			point2[1] = points[label2].second;
			point3[0] = points[label3].first;
			point3[1] = points[label3].second;

			motion = calcMotion(point1, point2, point3);

			mX.put("", motion[0]); 
			mY.put("", motion[1]); 
			mRot.put("", motion[2]); 

			m.push_back(std::make_pair("", mX));
			m.push_back(std::make_pair("", mY));
			m.push_back(std::make_pair("", mRot));

			pt.get_child("subjects." + label + ".motion").push_back(std::make_pair("", m));
			mX.clear();
			mY.clear();
			mRot.clear();
			m.clear();
		}

    }
	points.clear();
	std::string file = filename.substr(0,filename.size()-4);
	std::string name = file + ".json";
	write_json( name, pt);
}

void open_folder(std:: string folder, boost::property_tree::ptree pt)
{
	path p(folder);
	if (exists(p))
	{
		if (is_directory(p))
		{
			directory_iterator end_iter;
			for (directory_iterator dir_itr(p); dir_itr != end_iter; ++dir_itr)
			{
				if (dir_itr->path().filename().extension().string() == ".c3d")
				{
					try 
					{
						do_work(dir_itr->path().string(), pt);
					}
					catch (UuIcsC3d::OpenError const &err) 
					{
						cerr<<"An error occurred while processing file "<< err.filename();
						return;
					}
				}
					
			}
		}
		else
		{
			std::cout<< p << "is not a folder";
			return;
		}
	} 
	else 
	{
		std::cout<< p << "does not exist";
		return;
	}
}

// The main function checks the number of arguments and deals with exceptions
int main(int argc, char* argv[])
{
    if (argc<3) 
	{
		cerr<<"Expected filename and folder as arguments.\n";
		return 1;
    }
	boost::property_tree::ptree pt;
	try 
	{
		// read the json file
		boost::property_tree::read_json(argv[1], pt);
	}
	catch(UuIcsC3d::OpenError const &err)
	{
		cerr<<"An error occurred while parsing the file "<<err.filename();
		return 2;
	}
    try 
	{
		open_folder(argv[2], pt);
    } 
	catch (UuIcsC3d::OpenError const &err) 
	{
		cerr<<"An error occurred while opening folder "<<err.filename();
		return 2;
    }
    return 0;
}

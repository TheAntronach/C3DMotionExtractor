// C3DMotionExtractor.cpp : Defines the entry point for the console application.
//

/**
  * \file
  * \brief A commented example file; a good starting point learning the C3d library.
  */

/*
A C3D file contains a number of frames (time steps) and labels.
In every frame a label can either be not present (invalid), measured
(detected by hardware) or generated (e.g., filled in by interpolation).
This program counts those three possibilities and displays the totals for
every label.
*/

#include "stdafx.h"
#include "uuc3d.hpp"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <map>

using std::cerr;
using std::vector;
using UuIcsC3d::SpacePaddedString;
using UuIcsC3d::DataPoint3d;
using namespace boost::filesystem;


vector<float> calcMotion(vector<float> point1, vector<float> point2, vector<float> point3)
{
	vector<float> motion(3);

	float center23X = (point2[0] + point3[0]) / 2;
	float center23Y = (point2[1] + point3[1]) / 2;
	float centerX = (point1[0] + center23X) / 2;
	float centerY = (point1[1] + center23Y) / 2;
	float rot = atan2f(centerY, centerX);

	motion[0] = centerX;
	motion[1] = centerY;
	motion[2] = rot;
	
	return motion;
}

void do_work(std::string filename, boost::property_tree::ptree pt)
{
    // Open the file and read the header information
    // Will throw OpenError if reading is not possible.
    UuIcsC3d::C3dFileInfo fi(filename);
    // Every frame contains the same number of points.
    int ppf = fi.points_per_frame(); 
    if (ppf <= 0)
	{
		return;
	}
    int fc = fi.frame_count();
	std::cout << "frames: " << fc << "\n";

	// create the tree to parse to json format. 
	boost::property_tree::ptree m, mX, mY, mRot;
	

    // Get the labels. Not every point needs to have a label name.
    // We add dummy labels if necessary to supply a name for every point.
    vector<SpacePaddedString> labels(fi.point_labels());
    for (int i=labels.size(); i<ppf; ++i) 
	{
		labels.push_back(SpacePaddedString("<no label>"));
    }

    // Declare the vectors that maintain the count for each point (label).
    vector<int> invalid(ppf, 0), generated(ppf, 0), measured(ppf, 0);

    // Open the C3D file
    std::auto_ptr<UuIcsC3d::C3dFile> filep(fi.open());

    // Read every frame in variable frame_data and process it
    UuIcsC3d::FrameData frame_data;
	std::size_t found;

	int counter = 0;
	vector<float> motion(3);
	bool test = true;

	std::map<std::string, std::pair<float,float>> points;
    for (int i=0; i<fc; ++i)
	{
		filep->get_frame_data(frame_data, i);
		vector<float> point1(2), point2(2), point3(2);

		// For every point in the frame record its type
		for (int j=0; j<ppf; ++j) 
		{
			DataPoint3d const &dp=frame_data.points[j];
			if (labels[j].stripped().find("*") != 0) 
			{
				points[labels[j].stripped()].first = dp.x();
				points[labels[j].stripped()].second = dp.y();
			}
		}
		vector<float> motion(3);
		std::string label, label1, label2, label3;
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
    // report the counts for every label
	std::cout << "labels: \n";
    for (int i=0; i<ppf; ++i) 
	{
		std::cout <<labels[i].stripped()<<"\n";
    }
	std::string file = filename.substr(0,filename.size()-4);
	std::string name = "test1.json";
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

boost::property_tree::ptree parseJson(std::string filename)
{
	// create the tree to parse to json format. 
	boost::property_tree::ptree pt;
	boost::property_tree::read_json(filename, pt);
	boost::property_tree::ptree motions;
	pt.put("subjects.AA.age",155);
	/*
	int age = pt.get<int>("subjects.AA.age");
	std::cout << "Harald's leeftijd is " << age << std::endl;

	for(auto keyval : pt.get_child("subjects")) {
		std::cout << "   - " << keyval.first
			<< " = " << keyval.second.get<int>("age") << std::endl;
	}
	*/
	//std::string name = "test.json";
	//write_json( name, pt);
	return pt;
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
		boost::property_tree::read_json(argv[1], pt);
		std::map<std::string, vector<int>> points;
		std::cout<<"label: " << points["AA"][2] << std::endl;
		//pt.put("subjects.AA.motion",155);
		//std::string name = "test.json";
	    //write_json( name, pt);
	}
	catch(UuIcsC3d::OpenError const &err)
	{
		cerr<<"An error occurred while parsing the file "<<err.filename();
		return 2;
	}
    try 
	{
		//open_folder(argv[2], pt);
    } 
	catch (UuIcsC3d::OpenError const &err) 
	{
		cerr<<"An error occurred while opening folder "<<err.filename();
		return 2;
    }
    return 0;
}


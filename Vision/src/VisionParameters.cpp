#include "VisionParameters.h"

#include <string>
#include "ros/package.h"

std::string VisionParameters::defaultDataset = ros::package::getPath("Vision") + "/bin/DataSet.dts";
std::string VisionParameters::defaultClassifier =  ros::package::getPath("Vision") + "/bin/classifier[5-17].kcc";

void VisionParameters::buildClassifier()
{
	cd.load(dataset.c_str());
	cout << "----> Color Data set  [OK]" << "loaded from file:" << dataset << endl;
	//Creazione del classificatore colore
	cc.fast_build(&cd, 5, 17, false);
	cc.save_matrix(classifier.c_str());
}

void VisionParameters::loadClassifier()
{
	cout << "----> No classifier loaded, loading default classifier" << endl;
	cc.load_matrix(classifier.c_str());
	cout << "----> Classifier loaded  [OK]"<<endl;
}

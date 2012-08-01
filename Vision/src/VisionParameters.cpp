#include "VisionParameters.h"

char VisionParameters::defaultDataset[] = "DataSet.dts";
char VisionParameters::defaultClassifier[] = "classifier[5-17].kcc";

void VisionParameters::buildClassifier()
{
	cd.load(dataset);
	cout << "----> Color Data set  [OK]" << "loaded from file:" << dataset << endl;
	//Creazione del classificatore colore
	cc.fast_build(&cd, 5, 17, false);
	cc.save_matrix(classifier);
}

void VisionParameters::loadClassifier()
{
	cout << "----> No classifier loaded, loading default classifier" << endl;
	cc.load_matrix(classifier);
	cout << "----> Classifier loaded  [OK]"<<endl;
}

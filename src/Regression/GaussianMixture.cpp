//
//
//  @ Project : GaussianMixture library Untitled
//  @ File Name : GaussianMixture.cpp
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//

#include "Regression/GaussianMixture.h"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <iostream>

/// A single instance of Gaussian Mixture
GaussianMixture::Ptr gaussianMixture;

///constructor
GaussianMixture::GaussianMixture(GaussianMixture::Config _config): Regression("Gaussian Mixture", TYPE_GAUSSIAN_MIXTURE), config(_config),
    bestFitnessRecorder(-1,"bestFitness","'-r'"), averageFitnessRecorder(-1,"averageFitness","'-g'"), populationSizeRecorder(-1,"populationSize","'-b'"), epochRecorder(-1,"epoch.m","'-k'"){
    currentEpoch=0;
    bestFitness=double(1e34);

    if (config.methodName=="CMAES"){
        optimizer = createCMAES(config.optimizerConfig);
    }
    else
        optimizer = createCMAES(config.optimizerConfig);
}

///constructor
GaussianMixture::GaussianMixture(void): Regression("Gaussian Mixture", TYPE_GAUSSIAN_MIXTURE),
    bestFitnessRecorder(-1,"bestFitness","'-r'"), averageFitnessRecorder(-1,"averageFitness","'-g'"), populationSizeRecorder(-1,"populationSize","'-b'"), epochRecorder(-1,"epoch.m","'-k'"){
    currentEpoch=0;
    bestFitness=double(1e34);

    if (config.methodName=="CMAES"){
        optimizer = createCMAES(config.optimizerConfig);
    }
}

///destructor
GaussianMixture::~GaussianMixture(){
}

///config class construction
GaussianMixture::Config::Config(std::string configFilename){
    setlocale(LC_NUMERIC,"C");
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load Gaussian Misture config file.\n";

    tinyxml2::XMLElement * params = config.FirstChildElement( "GaussianMixture" );
    std::string method(params->FirstChildElement( "method" )->GetText());
    methodName = method;
    params->FirstChildElement( "parameters" )->QueryIntAttribute("inputsNo", &inputsNo);
    params->FirstChildElement( "parameters" )->QueryIntAttribute("outputsNo", &outputsNo);
    params->FirstChildElement( "parameters" )->QueryIntAttribute("populationSize", &populationSize);
    params->FirstChildElement( "parameters" )->QueryIntAttribute("gaussiansNo", &gaussiansNo);
    params->FirstChildElement( "parameters" )->QueryIntAttribute("maxIterations", &maxIterations);
    params->FirstChildElement( "parameters" )->QueryIntAttribute("trainSize", &trainSize);
    params->FirstChildElement( "parameters" )->QueryIntAttribute("testSize", &testSize);
    params->FirstChildElement( "parameters" )->QueryIntAttribute("outputType", &outputType);
    params->FirstChildElement( "parameters" )->QueryBoolAttribute("normalizeOutput", &normalizeOutput);

    std::string optConfig(params->FirstChildElement( "CMAESconfig" )->GetText());
    optimizerConfig = optConfig;

    //tinyxml2::XMLElement * bounds = config.FirstChildElement( "Boundaries" );
    for (int i=0;i<inputsNo;i++){
        //std::string paramName = "lambda" + std::to_string(i);
        //double min, max;
        //bounds->FirstChildElement( paramName.c_str() )->QueryDoubleAttribute("min", &min);
        //bounds->FirstChildElement( paramName.c_str() )->QueryDoubleAttribute("max", &max);
        boundaries.push_back(std::make_pair(0,1));
    }
    params = config.FirstChildElement( "trainingSet" );
    trainFilename = params->GetText();
    params = config.FirstChildElement( "testSet" );
    testFilename = params->GetText();
    params = config.FirstChildElement( "verificationSet" );
    verifFilename = params->GetText();

//    std::cout << "Configuration:\n";
//    std::cout << "Inputs no (dim): " << inputsNo << "\n";
//    std::cout << "Outputs no (dim): " << outputsNo << "\n";
//    //std::cout << "Search method: " << searchMethod << "\n";
//    std::cout << "Population size: " << populationSize << "\n";
//    std::cout << "Gaussians no: " << gaussiansNo << "\n";
//    std::cout << "Max iterations: " << maxIterations << "\n";
//    std::cout << "Training vector size: " << trainSize << "\n";
//    std::cout << "Test vector size: " << testSize << "\n";
//    std::cout << "Output type: " << outputType << "\n";
//    std::cout << "Normalize output: " << normalizeOutput << "\n";
    /*std::cout << "Gaussian boundaries: \n";
    int dimNo=0;
    for (auto it = boundaries.begin(); it!=boundaries.end(); it++){
        std::cout << "dim " << dimNo << ": min: " << it->first << ", max: " << it->second << "\n";
        dimNo++;
    }*/
}

/// compute cost for the given solution
double GaussianMixture::computeCost(const Eigen::VectorXd& x){
    size_t individualNo=0;
    tempCoef = Eigen::MatrixXd::Zero(population.getGaussiansNo((unsigned int)individualNo), config.outputsNo);
    setSolution(individualNo, x);
    double cost = computeIndividualFitness(features, output, tempCoef, (unsigned int)individualNo);//and fitness value
    std::cout << "cost " << cost << "\n";
    return cost;
}

/// set chromosome of the individual
void GaussianMixture::setSolution(size_t individualNo, const Eigen::VectorXd& x){
    population.setIndividual(x, individualNo);
}

/// optimize GM
bool GaussianMixture::optimizeGM(){
    //optimization
    std::string function("computeCost");
    std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap =
    {
        { "computeCost", std::bind(&GaussianMixture::computeCost,this,std::placeholders::_1)}
    };
    optimizer->Optimize(funcMap, function);
//    Eigen::VectorXd xmin;
//    optimizerPosture->getResult(xmin);

    return true;
}

/// Initialize training
void GaussianMixture::initializeTraining(void){
    readTrainingData(config.trainFilename);//read training data
    readVerificationData(config.verifFilename);//read verification data
    readTestData(config.testFilename);//read test data
    setBoundaries();
    setParameters();
    initializeMatrices();// initialize matrices
}

/// Initialize training
void GaussianMixture::initializeTraining(const std::string& trainFilename, const std::string& verifFilename, const std::string& testFilename){
    config.trainFilename = trainFilename;
    config.verifFilename = verifFilename;
    config.testFilename = testFilename;
    initializeTraining();
}

/// set parameters of polynomial
void GaussianMixture::setParameters() {
    polyCoef = Eigen::MatrixXd::Zero(config.gaussiansNo, config.outputsNo);
    population.setParameters( config.populationSize, config.gaussiansNo, config.inputsNo, config.outputsNo);
    bestPolynomial.setParameters(config.gaussiansNo, config.inputsNo, config.outputsNo);
}

/// set boundaries of function's parameters
void GaussianMixture::setBoundaries(){
    computeBoundaries(config.boundaries, inputDomain);
    population.setBoundaries(config.boundaries,inputDomain);
    bestPolynomial.setBoundaries(config.boundaries,inputDomain);
    if (config.methodName=="CMAES"){
        std::vector<std::pair<double,double>> range;
        range.resize(config.gaussiansNo*config.inputsNo*2);
        for (size_t gaussNo=0;gaussNo<(size_t)config.gaussiansNo;gaussNo++){
            for (size_t dimNo=0;dimNo<(size_t)config.inputsNo;dimNo++){
                range[gaussNo*(config.inputsNo*2)+dimNo*2+0].first = 0;
                range[gaussNo*(config.inputsNo*2)+dimNo*2+0].second = 1;
                range[gaussNo*(config.inputsNo*2)+dimNo*2+1].first = config.boundaries[dimNo].first;
                range[gaussNo*(config.inputsNo*2)+dimNo*2+1].second = config.boundaries[dimNo].second;
            }
        }
        optimizer->setBoundaries(range);
    }
}

/// compute n boundaries (width of Gaussian function)
void GaussianMixture::computeBoundaries(std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain){
    for (size_t i=0;i<_gaussianBoundaries.size();i++){
          double width=_inputDomain[i].second-_inputDomain[i].first;
          if (width==0)
              _gaussianBoundaries[i].second=std::numeric_limits<double>::max();
          else
            _gaussianBoundaries[i].second=1/(2*pow(width/(10*4.2919),2.0));
    }
}

/// read training data
void GaussianMixture::readTrainingData(const std::string& filename){
    int length;
    Eigen::MatrixXd featuresTmp = Eigen::MatrixXd::Zero(config.trainSize, config.inputsNo);
    Eigen::MatrixXd outputTmp = Eigen::MatrixXd::Zero(config.trainSize, config.outputsNo);
    //readInOutSingleRow(filename, featuresTmp, outputTmp, length);
    readInOutData(filename, featuresTmp, outputTmp, length);
    features = Eigen::MatrixXd::Zero(length, config.inputsNo);
    output = Eigen::MatrixXd::Zero(length, config.outputsNo);
    for (int i=0;i<length;i++){
        for (int j=0;j<config.inputsNo;j++){
            features(i,j) = featuresTmp(i,j);
        }
        for (int j=0;j<config.outputsNo;j++){
            output(i,j) = outputTmp(i,j);
        }
    }
    if (length<config.trainSize){
        config.trainSize=length;
        std::cout << "Dataset train size: " << config.trainSize << "\n";
    }
	
    inputDomain.resize(config.inputsNo);
    outputDomain.resize(config.outputsNo);
    // set a boundaries
    std::vector<double>values(config.trainSize, 0);
    for(int i=0;i<config.inputsNo;i++){
        for(int j = 0; j<config.trainSize;j++){
          values[j]=features(j,i);
        }
        inputDomain[i].first = *std::min_element(values.begin(), values.end());
        inputDomain[i].second = *std::max_element(values.begin(), values.end());
    }
    for(int i=0;i<config.outputsNo;i++){
        for(int j = 0; j<config.trainSize;j++){
          values[j]=output(j,i);
        }
        outputDomain[i].first = *std::min_element(values.begin(), values.end());
        outputDomain[i].second = *std::max_element(values.begin(), values.end());
    }
    if (config.normalizeOutput){
        for(int i=0;i<config.outputsNo;i++){
            for(int j = 0; j<config.trainSize;j++){
                output(j,i) = normalizeValue(output(j,i),outputDomain[i]);
            }
        }
    }
}

/// normalize output
double GaussianMixture::normalizeValue(double value, const std::pair<double,double>& minMax) const{
    return (value-minMax.first)/(minMax.second-minMax.first);
}

/// denormalize output
double GaussianMixture::denormalizeValue(double value, const std::pair<double,double>& minMax) const{
    return value*(minMax.second-minMax.first)+minMax.first;
}

/// read verification data
void GaussianMixture::readVerificationData(const std::string& filename){
    int length;
    Eigen::MatrixXd verifInputTmp = Eigen::MatrixXd::Zero(config.testSize, config.inputsNo);
    Eigen::MatrixXd verifOutputTmp = Eigen::MatrixXd::Zero(config.testSize, config.outputsNo);
    //readInOutSingleRow(filename, verifInputTmp, verifOutputTmp, length);
    readInOutData(filename, verifInputTmp, verifOutputTmp, length);
//    std::cout << "legnth " << length << "\n";
//    std::cout << "verifOutputTmp " << verifOutputTmp(0,0) << "\n";
    verifInput = Eigen::MatrixXd::Zero(length, config.inputsNo);
    verifOutput = Eigen::MatrixXd::Zero(length, config.outputsNo);
    for (int i=0;i<length;i++){
        for (int j=0;j<config.inputsNo;j++){
            verifInput(i,j) = verifInputTmp(i,j);
//            std::cout << "in " << verifInput(i,j) << ", ";
        }
//        std::cout << "out \n";
        for (int j=0;j<config.outputsNo;j++){
            verifOutput(i,j) = verifOutputTmp(i,j);
//std:cout << verifOutput(i,j) << "\n";
        }
//        getchar();
    }
    if (length<config.testSize){
        config.testSize=length;
        std::cout << "Dataset verification size: " << config.testSize << "\n";
    }
    if (config.normalizeOutput){
        for(int i=0;i<config.outputsNo;i++){
            for(int j = 0; j<config.testSize;j++){
                verifOutput(j,i) = normalizeValue(verifOutput(j,i),outputDomain[i]);
            }
        }
    }
}

/// read train/test/verification data
void GaussianMixture::readInOutData(const std::string& filename, Eigen::MatrixXd& _input, Eigen::MatrixXd& _output, int& vecLength){
    std::ifstream infile(filename);
    std::string line;
    vecLength = 0;
    int lineNo=0;
    while (std::getline(infile, line)) {
        vecLength++;
        if (lineNo>=_input.rows())//divided by 2 because we increment for input and for output line
            break;
        std::istringstream iss(line); //input
        int colNo=0; double value;
        while (iss >> value) {
            if (vecLength%2==0){
                _output(lineNo,colNo) = value;
                lineNo++;
            }
            else {
                _input(lineNo,colNo) = value;
            }
            if ((iss.peek() == ',')||(iss.peek() == ' '))
                iss.ignore();
            colNo++;
        }
    }
    vecLength = lineNo;
}

///// read train/test/verification data
//void GaussianMixture::readInOutSingleRow(const std::string& filename, Eigen::MatrixXd& _input, Eigen::MatrixXd& _output, int& vecLength){
//    std::ifstream infile(filename);
//    std::string line;
//    vecLength = 0;
//    int lineNo=0;
//    while (std::getline(infile, line)) {
//        if (lineNo>=_input.rows())
//            break;
//        std::istringstream iss(line); //input
//        int colNo=0; double value;
//        double isCorrect=false;
//        while (iss >> value) {
//            isCorrect=true;
//            _input(lineNo,colNo) = value;
////            if (colNo<520){
////                if (value==100)
////                    value=10;
////                else
////                    isCorrect=true;
////                _input(lineNo,colNo) = value;
//                //std::cout << "input" << colNo << " " << value <<"\n";
//                //getchar();
////            }
////            else if (colNo==520||colNo==521){
////                _output(lineNo,colNo-520) = value;
//////                std::cout << "output" << colNo << " " << value <<"\n";
//////                getchar();
////            }
////            else if (colNo==522){
////                if (value!=2)
////                    isCorrect=false;//ignore second floor
////            }
////            else if (colNo==523){
////                if (value!=1)
////                    isCorrect=false;//ignore building 1
////            }
//            if ((iss.peek() == ',')||(iss.peek() == ' '))
//                iss.ignore();
//            colNo++;
//        }
//        if (isCorrect){
//            vecLength++;
//            lineNo++;
//        }
//    }
//    std::cout << "Examples no: " << vecLength << "\n";
//}

/// read test data
void GaussianMixture::readTestData(const std::string& filename){
    int length;
    Eigen::MatrixXd testInputTmp = Eigen::MatrixXd::Zero(config.testSize, config.inputsNo);
    Eigen::MatrixXd testOutputTmp = Eigen::MatrixXd::Zero(config.testSize, config.outputsNo);
    //readInOutSingleRow(filename, testInputTmp, testOutputTmp, length);
    readInOutData(filename, testInputTmp, testOutputTmp, length);
    testInput = Eigen::MatrixXd::Zero(length, config.inputsNo);
    testOutput = Eigen::MatrixXd::Zero(length, config.outputsNo);
    for (int i=0;i<length;i++){
        for (int j=0;j<config.inputsNo;j++)
            testInput(i,j) = testInputTmp(i,j);
        for (int j=0;j<config.outputsNo;j++)
            testOutput(i,j) = testOutputTmp(i,j);
    }
    if (length<config.testSize){
        config.testSize=length;
        std::cout << "Dataset test size: " << config.testSize << "\n";
    }
    if (config.normalizeOutput){
        for(int i=0;i<config.outputsNo;i++){
            for(int j = 0; j<config.testSize;j++){
                testOutput(j,i) = normalizeValue(testOutput(j,i),outputDomain[i]);
            }
        }
    }
}

/// initialize matrices
void GaussianMixture::initializeMatrices(){
//    _sample = Eigen::MatrixXd::Zero(1, config.inputsNo);
}

/// compute output for the best polynomial
double GaussianMixture::computeOutput(const Eigen::MatrixXd& input, int outNo) const{
//    std::cout << "bestPolynomial.computeValue(input, polyCoef, outNo) " << bestPolynomial.computeValue(input, polyCoef, outNo) << "\n";
    double out;
    if (config.normalizeOutput){
        out = denormalizeValue(bestPolynomial.computeValue(input, polyCoef, outNo),outputDomain[outNo]);
    }
    else
        out = bestPolynomial.computeValue(input, polyCoef, outNo);
    return out;
}

/// compute gradient of trained function
void GaussianMixture::computeGradient(const Eigen::MatrixXd& input, Eigen::MatrixXd& grad) const{
    bestPolynomial.computeGradient(input, grad, polyCoef, 0);
}

/// compute individual fitness
double GaussianMixture::computeIndividualFitness(const Eigen::MatrixXd& _input, const Eigen::MatrixXd& _output, Eigen::MatrixXd& _coef, unsigned int individualNo){
    double fitness;
    if (!computeCoef(individualNo,_coef)){
		fitness =1e34;
	}
	else {
        //fitness = dim*log(population.computeIndividualFitness(features, output, coef, individual_no)/(double)(*features).rows())+0.25*population.population[individual_no]->getGaussiansNo();
        fitness = population.computeIndividualFitness(_input, _output, _coef, individualNo)/double(features.rows());
        fitness*=fitness;
        //fitness = 0.5*population.computeIndividualFitness(features, output, coef, individual_no)/(double)(*features).rows()+0.5*population.computeIndividualFitness(verifInput, verifOutput, coef, individual_no)/(double)(*verifInput).rows();
	}
    if (fitness<bestFitness){//the best individual
        bestFitness = fitness;
        bestIndividual = individualNo;
        population.setBestIndividualNo(bestIndividual);
        polyCoef = _coef;
        bestPolynomial = population.getIndividual(individualNo);
        bestPolynomial.setGaussianBoundaries(population.getGaussianBoundaries(individualNo));
	}
    return fitness;
}

/// search for the best Approximation function - PSO method
void GaussianMixture::train() {
    if (config.methodName=="CMAES"){
        optimizeGM();
    }
    else {
        for(int i=0;i<(int)population.getPopulationSize();i++){//fitness evaluation for every individual
            tempCoef = Eigen::MatrixXd::Zero(population.getGaussiansNo(i), config.outputsNo);
            double fit = computeIndividualFitness(features, output, tempCoef, i);
            std::cout.precision(std::numeric_limits<double>::max_digits10);
            std::cout << "Individual " << i << " fitness: " << fit << "\n";
        }
        population.computeAverageFitness();
        std:: cout << "epoch: " << currentEpoch << ", best fitness: " << bestFitness << "\n";
        //save initial values
        bestFitnessRecorder.store(currentEpoch, bestFitness);
        averageFitnessRecorder.store(currentEpoch,population.getAverageFitness());

        while (currentEpoch<(unsigned int)config.maxIterations){
            for(int i=0;i<(int)population.getPopulationSize();i++){//move individuals
                if (i!=(int)bestIndividual){
                    population.moveIndividual(i,bestPolynomial);
                }
                tempCoef = Eigen::MatrixXd::Zero(population.getGaussiansNo(i), 1);
                double fit = computeIndividualFitness(features, output, tempCoef, i);//and fitness value
                std::cout << "Individual " << i << " fitness: " << fit << "\n";
            }
            currentEpoch++;
            //save
            //if ((currentEpoch%(config.maxIterations/10))==0) {
                std::cout << "epoch: " << currentEpoch << ", best fitness: " << bestFitness << "\n";

            //}
            population.computeAverageFitness();
            bestFitnessRecorder.store(currentEpoch,bestFitness);
            averageFitnessRecorder.store(currentEpoch,population.getAverageFitness());
        }
        //save initial values
        bestFitnessRecorder.savePlot();
        averageFitnessRecorder.savePlot();
    }

    double maxerr, averr;
    computeAPE(features, output, averr, maxerr);
    std::cout << "Error for training data: average " << averr << ", max " << maxerr << "\n";
    computeAPE(testInput, testOutput, averr, maxerr);
    std::cout << "Error for test data: average " << averr << ", max " << maxerr << "%f\n";
    if (config.inputsNo<3)
      savePlotFile("plot_figure.m", features, output,  testInput, testOutput);
}

/// write summary
void GaussianMixture::writeSummary(const std::string& filenameTrain, const std::string& filenameTest){
    testResults(filenameTrain, features, output);
    testResults(filenameTest, testInput, testOutput);
}

/// compute coefficients
bool GaussianMixture::computeCoef(unsigned int individual_no, Eigen::MatrixXd& resultCoef){
    Individual individual = population.getIndividual(individual_no);
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(features.rows(), individual.getGaussiansNo());
    Eigen::MatrixXd Vprim = Eigen::MatrixXd::Zero(individual.getGaussiansNo(), individual.getGaussiansNo());
    Eigen::MatrixXd p = Eigen::MatrixXd::Zero(individual.getGaussiansNo(), config.outputsNo);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(individual.getGaussiansNo(), individual.getGaussiansNo());
//    Eigen::MatrixXd Ginv = Eigen::MatrixXd::Zero(individual.getGaussiansNo(), individual.getGaussiansNo());
    Eigen::MatrixXd _sample = Eigen::MatrixXd::Zero(1, config.inputsNo);
    for (int i=0;i<features.rows();i++){
        _sample = features.row(i);
        //std::cout << "sample " << sample << "\n";
        //getchar();
        for (int j=0;j<individual.getGaussiansNo();j++){
            V(i,j)=individual.computeValue(j,_sample);
            //std::cout << "V(i,j) " << V(i,j) << ", ";
        }
		//(*r)(i,0) = (*output)(i,0);
    }
    //std::cout << "\n";
    //getchar();
    Vprim = V.transpose();
    G=Vprim*V;
    p=Vprim*output;
    //cout << "G: " << G << endl <<" p: " <<p << endl <<" V: " << V << endl <<" Vprim: " << Vprim <<endl;
    //if (G.lu().isInvertible()){ // inversion successfull
	
		//using inverse of Gramm matrix
        //Ginv = G.inverse();
        //resultCoef = Ginv*p;
		//using LDLT decomposition
		//(*G).ldlt().solve(*p,result_coef); 
		//(*G).llt().solve(*p,result_coef); 
		// a little bit faster llt
        G.llt().solveInPlace(p);
        resultCoef=p;
		//LU
		//(*G).lu().solve(*p,result_coef);
		//SVD
        //(*G).svd().solve(*->gaussiansNo,p,result_coef);//so slooow
		
		//cout << "WEE"<< (*result_coef).rows()<<endl;
        /*for (int i=0;i<resultCoef.rows();i++){
          cout << "Result coeff " << resultCoef(i,0) << " ";
        }*/
		return true;
	//}
	//else
	//	return false; // inversion failed
}

/// save plot file (test of GaussianMixture function)
void GaussianMixture::savePlotFile(const std::string& filename, const Eigen::MatrixXd& _input, const Eigen::MatrixXd& _output, const Eigen::MatrixXd& _testInput, const Eigen::MatrixXd& _testOutput){
    std::ofstream ofstr;
    ofstr.open (filename);
    ofstr << "close all;\n";
    unsigned int iter=0;
    Eigen::MatrixXd _sample;
    for (int j=0;j<config.outputsNo;j++){
		iter=0;
        for (int i=0;i<_input.rows();i++){
            _sample = _input.row(iter);
            if (config.inputsNo==1){
              ofstr << "x=" <<_input(iter,0) << "; y=" << _output(iter,j) << ";\n";
              ofstr << "plot(x,y,'*r'); hold on;\n";
              ofstr << "x=" <<_input(iter,0) << "; y=" << computeOutput(_sample,j) << ";\n";
              ofstr << "plot(x,y,'ob'); hold on;\n";
			}
            else if (config.inputsNo==2){
              ofstr << "x=" << _input(iter,0) << "; y=" << _input(iter,1) << "; z=" << _output(iter,j) << ";\n";
              ofstr << "plot3(x,y,z,'*r'); hold on;\n";
              ofstr << "x=" << _input(iter,0) << "; y=" << _input(iter,1) << "; z=" << computeOutput(_sample,j) << ";\n";
              ofstr << "plot3(x,y,z,'ob'); hold on;\n";
			}
			iter++;
		}
	}
    iter=0;
    _sample = Eigen::MatrixXd::Zero(1, config.inputsNo);
    for (int j=0;j<config.outputsNo;j++){
		iter=0;
        for (int i=0;i< _testInput.rows();i++){
            _sample = _testInput.row(iter);
            if (config.inputsNo==1){
                ofstr << "x=" <<_testInput(iter,0) << "; y=" << _testOutput(iter,j) << ";\n";
                ofstr << "plot(x,y,'*g'); hold on;\n";
                ofstr << "x=" <<_testInput(iter,0) << "; y=" << computeOutput(_sample,j) << ";\n";
                ofstr << "plot(x,y,'ok'); hold on;\n";
			}
            else if (config.inputsNo==2){
                ofstr << "x=" << _testInput(iter,0) << "; y=" << _testInput(iter,1) << "; z=" << _testOutput(iter,j) << ";\n";
                ofstr << "plot3(x,y,z,'*g'); hold on;\n";
                ofstr << "x=" << _testInput(iter,0) << "; y=" << _testInput(iter,1) << "; z=" << computeOutput(_sample,j) << ";\n";
                ofstr << "plot3(x,y,z,'ok'); hold on;\n";
			}
			iter++;
		}
	}
    double arg[2];
    if (config.inputsNo==2)
        ofstr << "[X,Y]=meshgrid([" << inputDomain[0].first << ":" << (inputDomain[0].second-inputDomain[0].first)/100 << ":" << inputDomain[0].second << "],[" << inputDomain[1].first << ":" << (inputDomain[1].second-inputDomain[1].first)/100 << ":" << inputDomain[1].second << "]);\n";
	
    for (int j=0;j<config.outputsNo;j++){
		for (int i=0;i<100;i++){ 
            arg[0]=inputDomain[0].first+i*((inputDomain[0].second-inputDomain[0].first)/99);
            if (config.inputsNo==1){
                  _sample(0,0)=arg[0];
                  ofstr << "x(" << i+1 << ")=" << arg[0] << "; y(" << i+1 << ")=" << computeOutput(_sample,j) << ";\n";
			}
            else if (config.inputsNo==2){ //jakis mesh lub surf
				  for (int k=0;k<100;k++) {
                    arg[0]=inputDomain[0].first+i*((inputDomain[0].second-inputDomain[0].first)/99);
                    arg[1]=inputDomain[1].first+k*((inputDomain[1].second-inputDomain[1].first)/99);
                    _sample(0,0)=arg[0];
                    _sample(0,1)=arg[1];
                    ofstr << "Z(" << k+1 << "," << i+1 << ")=" << computeOutput(_sample,j) << ";\n";
				  }
			}	
		}
        ofstr << "plot(x,y,'b'); hold on;\n";
	}
    ofstr << "%Input domain: " << inputDomain[0].first << ", " << inputDomain[0].second << "\n";

    if (config.inputsNo==2) {
      ofstr << "X=X(1:1:100,1:1:100);\n";
      ofstr << "Y=Y(1:1:100,1:1:100);\n";
      ofstr << "mesh(X,Y,Z);\n";
	}
    ofstr.close();
}

/// test GaussianMixture function
void GaussianMixture::testResults(const std::string& filename, const Eigen::MatrixXd& _input, const Eigen::MatrixXd& _output){
    Eigen::MatrixXd _sample = Eigen::MatrixXd::Zero(1, config.inputsNo);
	int iter = 0;
    std::ofstream ofstr;
    ofstr.open (filename);
	int correct=0;
    for (int i=0;i< _input.rows();i++){
        _sample = _input.row(iter);
		
        ofstr << "f" << i << "(";
        for (int j=0;j<_sample.cols();j++)
            ofstr << _sample(0,j) << ",";
        ofstr << ") = (";
        for (int k=0;k<config.outputsNo;k++){
            ofstr << computeOutput(_sample,k) << ", ";
		}
        ofstr << ") should be (";
        for (int k=0;k<config.outputsNo;k++){
            if (config.normalizeOutput)
                ofstr << denormalizeValue(_output(iter,k),outputDomain[k]) << ", ";
            else
                ofstr << _output(iter,k) << ", ";
		}
        ofstr << ") error = (";
        for (int k=0;k<config.outputsNo;k++){
            if (config.normalizeOutput)
                ofstr << computeOutput(_sample,k)-denormalizeValue((_output)(iter,k),outputDomain[k]) << ", ";
            else
                ofstr << computeOutput(_sample,k)-(_output)(iter,k) << ", ";
		}
        ofstr << ")\n";
        double err=0;
        if (config.normalizeOutput)
            err = fabs(computeOutput(_sample,0)-denormalizeValue((_output)(iter,0),outputDomain[0]));
        else
            err = fabs(computeOutput(_sample,0)-(_output)(iter,0));
        if (err<0.5)
		  correct++;
		iter++;
	}
    ofstr << "correct = " << (double(correct)/double(iter))*100 << "%\n";
    for (int i=0;i<config.inputsNo;i++){
      ofstr << "nbound[" << i << "] = (" << bestPolynomial.getGaussianBoundaries()[i].first << "," << bestPolynomial.getGaussianBoundaries()[i].second << ")\n";
	}
    double maxerr, averr;
    computeAPE(_input, _output, averr, maxerr);
    ofstr << "Error: average " << averr << ", max " << maxerr << "\n";
    ofstr << "Poly elements number: " << bestPolynomial.getGaussiansNo() << "\n";
    ofstr.close();
    bestPolynomial.save2file(filename,polyCoef,config.outputType);
}

/// store results
void GaussianMixture::storeResult(std::string filename){
    bestPolynomial.store(filename,polyCoef,outputDomain);
}

/// store results
void GaussianMixture::load(std::string filename){
    bestPolynomial.load(filename, polyCoef, outputDomain);
}

/// test GaussianMixture function
double GaussianMixture::computeAPE(const Eigen::MatrixXd& _input, const Eigen::MatrixXd& _output, double& avError, double& maxError){
    Eigen::MatrixXd _sample = Eigen::MatrixXd::Zero(1, config.inputsNo);
	int iter = 0;
    double sum=0;
    avError=0;
    maxError=0;
    for (int i=0;i<_input.rows();i++){
        _sample = _input.row(iter);
        for (int k=0;k<config.outputsNo;k++){
          double error;
          if (config.normalizeOutput){
              error = fabs(denormalizeValue(_output(iter,k),outputDomain[k])-computeOutput(_sample,k));
          }
          else
              error = fabs(_output(iter,k)-computeOutput(_sample,k));
          sum+=error;///abs(_output(iter,k));
          if (error>maxError)
            maxError=error;
          avError+=error;
		}
		iter++;
	}
    avError=avError/double(iter);
    std::cout << "average_error=" << avError << " max_error=" << maxError << "\n";
    return (sum/double(iter))*100.0;
}

std::unique_ptr<regression::Regression> regression::createGaussianApproximation(GaussianMixture::Config config) {
    return walkers::make_unique<GaussianMixture>(config);
}

std::unique_ptr<regression::Regression> regression::createGaussianApproximation(void) {
    return walkers::make_unique<GaussianMixture>();
}

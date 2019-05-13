//
//
//  @ Project : Approximation library Untitled
//  @ File Name : Individual.cpp
//  @ Date : 2009-07-16
//  @ Author : Dominik Belter
//
//

#include "Regression/individual.h"
#include <chrono>
#include <random>
#include <fstream>
#include <stdexcept>
#include <exception>

/// default constructor
Individual::Individual(){
    fitnessValue=std::numeric_limits<double>::max();
    maxChange=0.25;
    bestFitnessValue=std::numeric_limits<double>::max();
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator.seed(seed);
}

/// default destructor
Individual::~Individual(){
}

/// set parameters
void Individual::setParameters(unsigned int _gaussiansNo, unsigned int _dim, unsigned int _outputsNo){
    gaussiansNo = _gaussiansNo;
	dim = _dim;
    outputsNo = _outputsNo;
    createChromosome();
    c1=2;
    c2=2;
    maxChange=0.25;
}

/// set boundaries of function's parameters
void Individual::setBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries, const std::vector<std::pair<double,double>>& _inputDomain){
    gaussianBoundaries = _gaussianBoundaries;
    inputDomain = _inputDomain;
    gaussianBoundariesChange.resize(gaussianBoundaries.size());
    bestGaussianBoundaries.resize(gaussianBoundaries.size());
    for (size_t i=0;i<gaussianBoundaries.size();i++){
        gaussianBoundariesChange[i]=0;
        std::uniform_real_distribution<double> distribution (gaussianBoundaries[i].first,gaussianBoundaries[i].second);
        gaussianBoundaries[i].second=distribution(generator);
        bestGaussianBoundaries[i]=gaussianBoundaries[i].second;
    }
}

/// set boundaries of function's parameters
void Individual::setGaussianBoundaries(const std::vector<std::pair<double,double>>& _gaussianBoundaries){
    gaussianBoundaries = _gaussianBoundaries;
}

/// create gene
void Individual::initializeGaussian(int gaussianNo) {
    chromosome[gaussianNo].setParameters(dim);
    chromosome[gaussianNo].setBoundaries(gaussianBoundaries,inputDomain);

    bestPosition[gaussianNo].setParameters(dim);
    bestPosition[gaussianNo].setBoundaries(gaussianBoundaries,inputDomain);
}

/// create chromosome
void Individual::createChromosome() {
    chromosome.resize(gaussiansNo);
    bestPosition.resize(gaussiansNo);
    for (int i =0;i<(int)gaussiansNo;i++) {
        initializeGaussian(i);
        //chromosome[i].setBoundaries(gaussianBoundaries, inputDomain);
        //bestPosition[i].setBoundaries(gaussianBoundaries, inputDomain);
	}

    for (int i =0;i<(int)gaussiansNo;i++) {
        bestPosition[i] = chromosome[i];
	}
}

//PSO change the best position
void Individual::changeBestPosition(){
    for (int i =0;i<(int)gaussiansNo;i++) {
        bestPosition[i] = chromosome[i];
	}
    for (size_t i=0;i<dim;i++) {
      bestGaussianBoundaries[i] = gaussianBoundaries[i].second;
    }
}

/// compute value of polynomial represented by individual; point must be a column vector
double Individual::computeValue(const Eigen::MatrixXd& point, const Eigen::MatrixXd& coefficient, int outNo) const{
    double result=0;
    for (int i =0;i<coefficient.rows();i++) {
        double c=coefficient(i,outNo);
/*		if ((_isnan(c))||(_finite(c))) c=0;
		char ch1[10],ch2[10];
        double dd = sqrt(-1.0);
		sprintf(ch1, "%g", dd);
		sprintf(ch2, "%g", c);
		if (strcmp(ch1,ch2)==0)
				c=0;*/
        result+=c * chromosome[i].computeValue(point);
	}
	return result;
}

/// compute gradient of polynomial represented by individual; point must be a column vector
void Individual::computeGradient(const Eigen::MatrixXd& point, Eigen::MatrixXd& grad, const Eigen::MatrixXd& coefficient, int outNo) const{
    grad.setZero();
    for (int i =0;i<coefficient.rows();i++) {
        double c=coefficient(i,outNo);
        Eigen::MatrixXd gradTmp(1,dim);
        chromosome[i].computeGradient(point, gradTmp);
        grad+=c * gradTmp;
    }
}

/// compute value of polynomial represented by individual; point must be a column vector
double Individual::computeValue(int gaussNo, const Eigen::MatrixXd& point) const{
    return chromosome[gaussNo].computeValue(point);
}

/// move individual
void Individual::moveIndividual(const Individual& bestParticle) {
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    for (size_t j=0;j<dim;j++) {
       //printf("%f, %f, %f, %f, %f, %f\n",c1,randdouble(0,1),(bestParticle.gaussianBoundaries)(j,1),gaussianBoundaries(j,1),c2,best_gaussianBoundaries[j]);
        gaussianBoundariesChange[j]=gaussianBoundariesChange[j]+(c1*distribution(generator)*(bestParticle.gaussianBoundaries[j].second-gaussianBoundaries[j].second))+(c2*distribution(generator)*(bestGaussianBoundaries[j]-gaussianBoundaries[j].second));
        if (std::isinf(gaussianBoundariesChange[j]))
            gaussianBoundariesChange[j]=0;
        gaussianBoundaries[j].second+=gaussianBoundariesChange[j];
        if (std::isinf(gaussianBoundaries[j].second))
            gaussianBoundaries[j].second = std::numeric_limits<double>::max();
        if(gaussianBoundaries[j].second<0)//reset
             gaussianBoundaries[j].second=bestParticle.gaussianBoundaries[j].second;
            //printf("%f, %f\n",gaussianBoundaries(j,1),gaussianBoundaries_change[j]);
    }
    for (int i=0;i<gaussiansNo;i++) {
          for (size_t j=0;j<dim;j++) {
                if (chromosome[i].getWidth(j)>(gaussianBoundaries[j].second)){
                    std::uniform_real_distribution<double> distributionN(gaussianBoundaries[j].first, gaussianBoundaries[j].second);
                    chromosome[i].setWidth(j,distributionN(generator));
                }
          }
    }
    if (gaussiansNo!=bestParticle.gaussiansNo){
        std::cout << "something is wrong no of gaussians differss\n";
        getchar();
    }
    for (int i=0;i<gaussiansNo;i++) {
        if (i<bestParticle.gaussiansNo) {
            for (size_t j=0;j<dim;j++) {
                chromosome[i].setCentroidChange(j,chromosome[i].getCentroidChange(j)+c1*distribution(generator)*(bestParticle.chromosome[i].getCentroid(j)-chromosome[i].getCentroid(j))+c2*distribution(generator)*(bestPosition[i].getCentroid(j)-chromosome[i].getCentroid(j)));
                if (fabs(chromosome[i].getCentroidChange(j))>((inputDomain[j].second-inputDomain[j].first)*maxChange)){
                    if (chromosome[i].getCentroidChange(j)<0)
                        chromosome[i].setCentroidChange(j,-(inputDomain[j].second-inputDomain[j].first)*maxChange);
                    else
                        chromosome[i].setCentroidChange(j, (inputDomain[j].second-inputDomain[j].first)*maxChange);
                }
                chromosome[i].setCentroid(j, chromosome[i].getCentroid(j) + chromosome[i].getCentroidChange(j));

                chromosome[i].setWidthChange(j,chromosome[i].getWidthChange(j)+c1*distribution(generator)*(bestParticle.chromosome[i].getWidth(j)-chromosome[i].getWidth(j))+c2*distribution(generator)*(bestPosition[i].getWidth(j)-chromosome[i].getWidth(j)));
                if (fabs(chromosome[i].getWidthChange(j))>(gaussianBoundaries[j].second-gaussianBoundaries[j].first)*maxChange){
                    if (chromosome[i].getWidthChange(j)<0)
                        chromosome[i].setWidthChange(j,-(gaussianBoundaries[j].second-gaussianBoundaries[j].first)*maxChange);
                    else
                        chromosome[i].setWidthChange(j, (gaussianBoundaries[j].second-gaussianBoundaries[j].first)*maxChange);
                }
                chromosome[i].setWidth(j,chromosome[i].getWidth(j) +chromosome[i].getWidthChange(j));
                if (chromosome[i].getWidth(j)<(gaussianBoundaries[j].first))
                    chromosome[i].setWidth(j,gaussianBoundaries[j].first);
                if (chromosome[i].getWidth(j)>(gaussianBoundaries[j].second))
                    chromosome[i].setWidth(j,gaussianBoundaries[j].second);
            }
	    }
	    else {
            for (size_t j=0;j<dim;j++) {
//                std::cout << "centroid before " << chromosome[i].getCentroid(j) << " centroid best " << bestPosition[i].getCentroid(j) << "\n";
                chromosome[i].setCentroidChange(j,chromosome[i].getCentroidChange(j)+c2*distribution(generator)*(bestPosition[i].getCentroid(j)-chromosome[i].getCentroid(j)));
                if (fabs(chromosome[i].getCentroidChange(j))>(inputDomain[j].second-inputDomain[j].first)*maxChange){
                    if (chromosome[i].getCentroidChange(j)<0)
                        chromosome[i].setCentroidChange(j,-(inputDomain[j].second-inputDomain[j].first)*maxChange);
                    else
                        chromosome[i].setCentroidChange(j,(inputDomain[j].second-inputDomain[j].first)*maxChange);
                }
                chromosome[i].setCentroid(j,chromosome[i].getCentroid(j)+chromosome[i].getCentroidChange(j));
//                std::cout << "centroid change " << chromosome[i].getCentroidChange(j) << " centroid after " << chromosome[i].getCentroid(j) << "\n";

//                std::cout << "width before " << chromosome[i].getWidth(j) << " width best " << bestPosition[i].getWidth(j) << "\n";
                chromosome[i].setWidthChange(j,chromosome[i].getWidthChange(j)+c2*distribution(generator)*(bestPosition[i].getWidth(j)-chromosome[i].getWidth(j)));
                if (fabs(chromosome[i].getWidthChange(j))>(gaussianBoundaries[j].second-gaussianBoundaries[j].first)*maxChange){
                    if (chromosome[i].getWidthChange(j)<0)
                        chromosome[i].setWidthChange(j,-(gaussianBoundaries[j].second-gaussianBoundaries[j].first)*maxChange);
                    else
                        chromosome[i].setWidthChange(j, (gaussianBoundaries[j].second-gaussianBoundaries[j].first)*maxChange);
                }
                chromosome[i].setWidth(j,chromosome[i].getWidth(j) + chromosome[i].getWidthChange(j));
                if (chromosome[i].getWidth(j)<(gaussianBoundaries[j].first))
                    chromosome[i].setWidth(j,gaussianBoundaries[j].first);
                if (chromosome[i].getWidth(j)>(gaussianBoundaries[j].second))
                    chromosome[i].setWidth(j,gaussianBoundaries[j].second);
//                std::cout << "width change " << chromosome[i].getWidthChange(j) << " centroid after " << chromosome[i].getWidth(j) << "\n";
            }
	    }
	}
}

/// compute fitness value
double Individual::computeFitness(const Eigen::MatrixXd& points, const Eigen::MatrixXd& expectedOutput, const Eigen::MatrixXd& polyCoef){
    double sum=0;
    for (int i=0;i<points.rows();i++){
        for (size_t j=0;j<outputsNo;j++){
			//point = (*points).block(i,0,1,(*points).cols())
            //sum += pow((*expected_output)(i,j)-computeValue(point,polyCoef,j),2.0);
            Eigen::MatrixXd point = points.row(i);
            sum += pow(expectedOutput(i,(int)j)-computeValue(point,polyCoef,(int)j),2.0);
		}
    }
	char ch1[20],ch2[20];
    double dd = sqrt(-1.0);
	sprintf(ch1, "%g", dd);
	sprintf(ch2, "%g", sum);
    if ((strcmp(ch1,ch2)==0)||(std::isnan(sum))||(sum!=sum))
			sum=1e10;
    if (sum<bestFitnessValue){
		changeBestPosition();
        bestFitnessValue=sum;
	}
    fitnessValue=sum;
	return sum;
}

/// get gene value
MultivariateGaussian Individual::getGeneValue(unsigned int gene_no){
	return chromosome[gene_no];
}

/// set individual
void Individual::setIndividual(const Eigen::VectorXd& x){
    for (size_t geneNo=0; geneNo<chromosome.size(); geneNo++){// gaussiansNo
        std::vector<double> parameters;
        parameters.resize(dim*2);
        for (size_t paramNo=0;paramNo<parameters.size();paramNo++){
            parameters[paramNo] = x(geneNo*dim*2+paramNo);
        }
        chromosome[geneNo].setGaussians(parameters);
    }
}

/// set gene value
void Individual::setGeneValue(const MultivariateGaussian& gene, unsigned int geneNo){
    chromosome[geneNo] = gene;
}

///save function to file
/// 1 - octave/matlab style
/// 2 - c++ style
void Individual::save2file(const std::string& filename, const Eigen::MatrixXd& coefficient, int type){
    std::ofstream ofstr;
    ofstr.open (filename, std::ofstream::out | std::ofstream::app);
	if (type==1) {
      for (size_t k=0;k<outputsNo;k++){
          ofstr << "\nf" << k << "(...) = ";
          for (int i =0;i<(int)gaussiansNo;i++) {
              double c =coefficient(i,k);
              if (c<0) ofstr << c << "*";
              else ofstr << "+" << c << "*";
              chromosome[i].save2file(ofstr,type);
	      }
          ofstr << "\n";
	  }
	}
	if (type==2) {
      for (size_t k=0;k<outputsNo;k++){
          ofstr << "\nf" << k << "(...) = ";
          for (int i =0;i<(int)gaussiansNo;i++) {
              double c =coefficient(i,k);
              if (c<0) ofstr << c << "*";
              else ofstr << "+" << c << "*";
              chromosome[i].save2file(ofstr,type);
	      }
          ofstr << "\n";
	  }
	}
    ofstr.close();
}

///store solution
void Individual::store(const std::string& filename, const Eigen::MatrixXd& coefficient, const std::vector<std::pair<double,double>>& outputDomain){
    std::ofstream ofstr;
    ofstr.open (filename, std::ofstream::out);
    ofstr.precision(24);
    ofstr << dim << "\n";
    ofstr << outputsNo << "\n";
    ofstr << gaussiansNo << "\n";
    for (size_t k=0;k<outputsNo;k++){
        ofstr << outputDomain[k].first << " " << outputDomain[k].second << "\n";
    }
    for (size_t k=0;k<outputsNo;k++){
        for (int i =0;i<(int)gaussiansNo;i++) {
             ofstr << coefficient(i,k) << " ";
        }
        ofstr << "\n";
    }
    for (int i=0;i<(int)gaussiansNo;i++) {
        for (size_t dimNo=0;dimNo<dim;dimNo++){
            ofstr << chromosome[i].getWidth(dimNo) << " ";
        }
        for (size_t dimNo=0;dimNo<dim;dimNo++){
            ofstr << chromosome[i].getCentroid(dimNo) << " ";
        }
        ofstr << "\n";
    }
    ofstr.close();
}

///store solution
void Individual::load(const std::string& filename, Eigen::MatrixXd& coefficient, std::vector<std::pair<double,double>>& outputDomain){
    std::string line;
    std::ifstream myfile(filename);
    if (myfile.is_open()){
        std::getline(myfile,line);
        std::istringstream iss(line);
        iss >> dim;
        std::getline(myfile,line);
        std::istringstream iss1(line);
        iss1 >> outputsNo;
        std::getline(myfile,line);
        std::istringstream iss2(line);
        iss2 >> gaussiansNo;
        outputDomain.resize(outputsNo);
        for (size_t k=0;k<outputsNo;k++){
            std::getline(myfile,line);
            std::istringstream iss3(line);
            iss3 >> outputDomain[k].first >> outputDomain[k].second;
        }
        coefficient = Eigen::MatrixXd(gaussiansNo,outputsNo);
//        std::cout << dim << ", " << outputsNo << ", " << gaussiansNo << "\n";
        for (size_t k=0;k<outputsNo;k++){
            std::getline(myfile,line);
            std::istringstream iss4(line);
            for (int i =0;i<(int)gaussiansNo;i++) {
                 iss4 >> coefficient(i,k);
//                 std::cout << "rcoeff " << coefficient(i,k) << "\n";
            }
        }
        chromosome.resize(gaussiansNo);
        for (int i=0;i<(int)gaussiansNo;i++) {
            std::getline(myfile,line);
            std::istringstream iss5(line);
            chromosome[i].setParameters(dim);
            for (size_t dimNo=0;dimNo<dim;dimNo++){
                double width;
                iss5 >> width;
                chromosome[i].setWidth(dimNo,width);
//                std::cout << "rwidth " << width << "\n";
            }
            for (size_t dimNo=0;dimNo<dim;dimNo++){
                double centroid;
                iss5 >> centroid;
                chromosome[i].setCentroid(dimNo,centroid);
//                std::cout << "rcentroid " << centroid << "\n";
            }
        }
        myfile.close();
    }
    else{
        throw std::runtime_error("Could not load Gaussian mixture model.\n");
    }
}

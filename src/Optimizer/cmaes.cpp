/// CMA-ES implementation
/// @author: Dominik Belter

#include "Optimizer/cmaes.h"
#include <chrono>

using namespace Eigen;

Cmaes::Ptr cmaesOptimizer;

Cmaes::Cmaes(void) : Optimizer("CMAES", TYPE_CMAES) {
}

Cmaes::Cmaes(std::string configFilename) : Optimizer("CMAES", TYPE_CMAES) {
    setlocale(LC_NUMERIC,"C");
    tinyxml2::XMLDocument doc;
    std::string filename = "../../resources/" + configFilename;
    doc.LoadFile( filename.c_str() );
    if (doc.ErrorID())
        std::cout << "unable to load optimizer config file: " << filename << ".\n";
    tinyxml2::XMLElement * model = doc.FirstChildElement( "CMAES" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose); //verbose
    model->FirstChildElement( "parameters" )->QueryIntAttribute("N", &N); //number of objective variables/problem dimension
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("sigma", &sigma); //coordinate wise standard deviation (step size)
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("stopfitness", &stopfitness); //stop if fitness < stopfitness (minimization)
    model->FirstChildElement( "parameters" )->QueryIntAttribute("breakIfNoProgressFor", &breakIfNoProgressFor); //stop if no progress for more than specified number of iterations
    range.resize(N);
    for (int dimNo=0;dimNo<N;dimNo++){
        std::string dimName = "dim" + std::to_string(dimNo);
        const tinyxml2::XMLElement* childNode = model->FirstChildElement(dimName.c_str());
        if (childNode != NULL){
            model->FirstChildElement( dimName.c_str() )->QueryDoubleAttribute("min", &range[dimNo].first);
            model->FirstChildElement( dimName.c_str() )->QueryDoubleAttribute("max", &range[dimNo].second);
        }
    }
}

const std::string& Cmaes::getName() const {
    return name;
}

/// set boundaries
void Cmaes::setBoundaries(const std::vector<std::pair<double,double>>& _range) {
    N = (int)_range.size();
    range = _range;
}

MatrixXd Cmaes::sort(MatrixXd& table, MatrixXd& index)
{
    MatrixXd j_i(1, table.size());
    j_i = table;
    std::sort(table.data(), table.data() + table.size());

    for (int i = 0; i < table.size(); i++)
        for (int j = 0; j < table.size(); j++)
            if (j_i(j) == table(0, i))
                index(i) = j;
    return table;
}

std::pair<MatrixXd, VectorXd> Cmaes::eig(const MatrixXd& A, const MatrixXd& _B) {
    Eigen::GeneralizedSelfAdjointEigenSolver<MatrixXd> solver(A, _B);
    MatrixXd V = solver.eigenvectors();
    VectorXd _D = solver.eigenvalues();
    return std::make_pair(V, _D);
}

void Cmaes::setParameters() {
    bestFitness = std::numeric_limits<double>::max();
    //Strategy parameter setting: Selection
    stopeval = 1000 * pow(N, 2); //stop after stopeval number of function evaluations
    std::srand((unsigned int)time(NULL));
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 mt(seed);
    xmean.resize(N);
    bestSolution.resize(N);
    if (injectSolutionFlag){
        xmean = injectedSolution;
    }
    for (int dimNo=0;dimNo<N;dimNo++){
        std::uniform_real_distribution<double> dist(range[dimNo].first, range[dimNo].second);
        xmean(dimNo) = dist(mt);
    }
    lambda = int(4 + floor(3*log(N))); //population size, offspring number
    mu = lambda / 2; //number of parents/points for recombination

    weights.resize(mu); //muXone array for weighted recombination
    for (int i = 0; i < mu; i++)
        weights(i) = log(mu + 0.5) - log(i + 1);

    double sum_weights = weights.sum(); //normalize recombination weights array
    weights /= sum_weights;

    double sum_weights_pow = 0; //variance-effectiveness of sum w_i x_i
    for (int i = 0; i < mu; i++)
        sum_weights_pow += pow(weights(i), 2);
    mueff = pow(weights.sum(),2.0) / sum_weights_pow;


    //Strategy parameter setting: Adaptation
    cc = (4 + (mueff / N)) / (N + 4 + (2 * (mueff / N))); //time constant for cumulation for C
    cs = (mueff + 2) / (N + mueff + 5); //t-const for cumulation for sigma control
    c1 = 2 / (pow((N + 1.3), 2) + mueff); //learning rate for rank-one update of C
    cmu = std::min(1 - c1, 2 * (mueff - 2 + (1 / mueff)) / (pow((N + 2), 2) + mueff)); //and for rank-mu update
    damps = 1 + 2 * std::max((double)0, sqrt((mueff - 1) / (N + 1)) - 1) + cs; //damping for sigma


    //Initialize dynamic (internal) strategy parameters and constants
    pc = VectorXd::Zero(N); //evolution paths for C and sigma
    ps = VectorXd::Zero(N);
    B = MatrixXd::Identity(N, N); //B defines the coordinate system
    D = VectorXd::Ones(N); //diagonal D defines the scaling
    C = B*MatrixXd::Identity(N, N)*B.transpose(); //covariance matrix C
    invsqrtC = B*MatrixXd::Identity(N, N)*B.transpose(); //C^-1/2
    chiN = pow(N, 0.5)*(1 - (1 / (4.0 * N)) + (1 / (21 * pow(N, 2)))); //expectation of ||N(0,I)|| == norm(randn(N,1))

    arindex.resize(1, lambda);
    arx.resize(N, lambda);
    arfitness.resize(1, lambda);
    arx_N_x_mu.resize(N, mu);
    eigeneval = 0; //track update of B and D
}

void Cmaes::genOffspring(std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap, std::string function) {
    //Generate and evaluate lambda offspring
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937 mt(seed);
    for (int k = 0; k < lambda; k++) {
        VectorXd rand_ = VectorXd::Ones(N);
        for (int dimNo=0;dimNo<N;dimNo++){
            std::normal_distribution<double> dist(0.0,1.0);
            rand_(dimNo)=dist(mt);
        }
        VectorXd arx_;
        if (injectSolutionFlag&&counteval==0){//inject solution
            arx_ = injectedSolution;
            injectSolutionFlag=false;
        }
        else{//normal mode
            rand_ = D.cwiseProduct(rand_);
            arx_ = xmean + sigma * B * (rand_);
        }
        for (int i=0;i<arx_.rows();i++){
            if (arx_(i)<range[i].first)
                arx_(i)=range[i].first;
            else if (arx_(i)>range[i].second)
                arx_(i)=range[i].second;
        }
        for (int i = 0; i < N; ++i)
            arx(i, k) = arx_(i);
        arfitness(k) = funcMap[function](arx_); //objective function call
        counteval++;
    }
}

void Cmaes::updateXmean() {
    //recombination, new mean value
    xold = xmean; //recombination, new mean value
    for (int i = 0; i < mu; ++i)
        for (int j = 0; j < N; ++j) {
            int a = int(arindex(i));
            arx_N_x_mu(j, i) = arx(j, a);
        }
    xmean = arx_N_x_mu*weights;
}

void Cmaes::updateEvoPath() {
    //Cumulation: Update evolution paths
    ps = (1 - cs)*ps + sqrt(cs*(2 - cs)*mueff)*invsqrtC*(xmean - xold) / sigma;
    ps_norm = ps.norm();
    double c__ = 1 - pow((1 - cs), (2 * counteval / lambda));
    if (((ps_norm / sqrt(c__)) / chiN) < (1.4 + (2.0 / (N + 1))))
        hsig = 1;
    else
        hsig = 0;
    pc = (1 - cc)*pc + hsig*sqrt(cc*(2 - cc)*mueff)*(xmean - xold) / sigma;
}

void Cmaes::updateMatrixC() {
    //Adapt covariance matrix C
    MatrixXd repmat(N, mu);
    for (int i = 0; i < mu; ++i)
        for (int j = 0; j < N; ++j)
            repmat(j, i) = xold(j, 0);

    MatrixXd artmp(N, mu);
    artmp = (1 / sigma) * (arx_N_x_mu - repmat);

    C = (1 - c1 - cmu)*C //regard old matrix
        + c1 * (pc*pc.transpose() //plus rank one update
        + (1 - hsig)*cc*(2 - cc)*C) //minor correction if hsig==0
        + cmu *artmp* weights.asDiagonal()*artmp.transpose(); //plus rank mu update
}

void Cmaes::updateOtherMatrices() {
    //Decomposition of C into B*diag(D.^2)*B' (diagonalization)
    eigeneval = counteval;
    MatrixXd CC(N, N);
    CC = C.triangularView<Eigen::Upper>();
    MatrixXd CCC(N, N);
    CCC = C.triangularView<Eigen::Upper>();

    for (int i = 0; i < N; ++i)
        CCC(i, i) = 0;
    C = CC + CCC.transpose(); //enforce symmetry

    std::pair<MatrixXd, VectorXd> BD;
    MatrixXd BB = MatrixXd::Identity(N, N);
    BD = eig(C, BB); //eigen decomposition, B==normalized eigenvectors
    B = BD.first;
    D = BD.second;

    for (int i = 0; i < D.size(); ++i)
        D(i) = sqrt(D(i)); //D is a vector of standard deviations now

    MatrixXd D_diag = MatrixXd::Zero(N, N);
    for (int i = 0; i < N; ++i)
        D_diag(i, i) = pow(D(i), -1);
    invsqrtC = B * D_diag * B.transpose();
}

void Cmaes::Optimize(std::map<std::string, std::function<double(const Eigen::VectorXd&)>>  funcMap, std::string function) {
    setParameters();
    //Generation Loop
    counteval = 0;
    while (counteval < stopeval){
        std::cout << counteval << "/" << stopeval << "\n";
        if (verbose)
            std::cout << "Iteration no: " << counteval << "\n";
        //Generate and evaluate lambda offspring
        genOffspring(funcMap, function);
        //Sort by fitness and compute weighted mean into xmean
        sort(arfitness, arindex); //minimization
        if (arfitness(0)<bestFitness){
            for (int dim=0;dim<N;dim++){
                bestSolution(dim) = arx(dim,int(arindex(0)));
            }
            noProgressIter=0;
            bestFitness = arfitness(0);
        }
        else {
            noProgressIter++;
        }

        if (verbose==2) {
            std::cout << "arfitness\n" << arfitness << "\n";
            std::cout << "arindex\n" << arindex << "\n";
            std::cout << "parameters matrix\n " << arx << "\n";
        }
        if (verbose==1){
            std::cout << "best parameters:\n";
            std::cout << bestSolution.transpose() << "\n";
            std::cout << "best fitness: " << bestFitness << "\n";
        }

        //recombination, new mean value
        updateXmean();
        if (verbose==2)
            std::cout << "xmean\n" << xmean << "\n";

        //Cumulation: Update evolution paths
        updateEvoPath();

        //Adapt covariance matrix C
        updateMatrixC();

        //Adapt step size sigma
        sigma *= exp((cs / damps)*((ps_norm / chiN) - 1));

        //Decomposition of C into B*diag(D.^2)*B' (diagonalization)
        if ((counteval - eigeneval) > (lambda / (c1 + cmu) / N / 10)) // to achieve O(N^2)
            updateOtherMatrices();

        if (noProgressIter>(size_t)breakIfNoProgressFor){
            noProgressIter=0;
            break;
        }

        //Break, if fitness is good enough or condition exceeds 1e14, better termination methods are advisable
        if (arfitness(0) <= stopfitness || D.maxCoeff()>(10000000 * D.minCoeff()))
            break;
    }
}

/// print output
void Cmaes::output() {
    //Return best point of last iteration.
    VectorXd xmin(N);
    for (int i = 0; i < N; ++i)
        xmin(i) = arx(i, (long int)arindex(0));
    std::cout << "min: " << xmin.transpose() << "\n";
}

/// get result
void Cmaes::getResult(Eigen::VectorXd& xmin){
    xmin = bestSolution;
}

std::unique_ptr<Optimizer> createCMAES(void){
    return walkers::make_unique<Cmaes>();
}

std::unique_ptr<Optimizer> createCMAES(std::string configFile){
    return walkers::make_unique<Cmaes>(configFile);
}

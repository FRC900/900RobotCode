// from https://raw.githubusercontent.com/Smorodov/Multitarget-tracker/master/HungarianAlg/HungarianAlg.h
#include <vector>
// http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm
class AssignmentProblemSolver
{
	private:
		// --------------------------------------------------------------------------
		// Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
		// --------------------------------------------------------------------------
		void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns) const;
		void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns) const;
		void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows) const;
		void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim) const;
		void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim) const;
		void step3 (int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim) const;
		void step4 (int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col) const;
		void step5 (int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim) const;
		// --------------------------------------------------------------------------
		// Computes a suboptimal solution. Good for cases with many forbidden assignments.
		// --------------------------------------------------------------------------
		void assignmentsuboptimal1(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns) const;
		// --------------------------------------------------------------------------
		// Computes a suboptimal solution. Good for cases with many forbidden assignments.
		// --------------------------------------------------------------------------
		void assignmentsuboptimal2(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns) const;
	public:
		enum TMethod { optimal, many_forbidden_assignments, without_forbidden_assignments };
		AssignmentProblemSolver();
		~AssignmentProblemSolver();
		double Solve(std::vector<std::vector<double> >& DistMatrix, std::vector<int>& Assignment,TMethod Method=optimal) const;
};

// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_ITERATIVE_SOLVER_BASE_H
#define EIGEN_ITERATIVE_SOLVER_BASE_H


/** \ingroup IterativeLinearSolvers_Module
  * \brief Base class for linear iterative solvers
  *
  * \sa class SimplicialCholesky, DiagonalPreconditioner, IdentityPreconditioner
  */
template< typename Derived>
class IterativeSolverBase
{
public:
  typedef typename internal::traits<Derived>::MatrixType MatrixType;
  typedef typename internal::traits<Derived>::Preconditioner Preconditioner;
  typedef typename MatrixType::Scalar Scalar;
  typedef typename MatrixType::Index Index;
  typedef typename MatrixType::RealScalar RealScalar;

public:

  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }

  /** Default constructor. */
  IterativeSolverBase()
    : mp_matrix(0)
  {
    init();
  }

  /** Initialize the solver with matrix \a A for further \c Ax=b solving.
    * 
    * This constructor is a shortcut for the default constructor followed
    * by a call to compute().
    * 
    * \warning this class stores a reference to the matrix A as well as some
    * precomputed values that depend on it. Therefore, if \a A is changed
    * this class becomes invalid. Call compute() to update it with the new
    * matrix A, or modify a copy of A.
    */
  IterativeSolverBase(const MatrixType& A)
  {
    init();
    compute(A);
  }

  ~IterativeSolverBase() {}

  /** Initializes the iterative solver with the matrix \a A for further solving \c Ax=b problems.
    *
    * Currently, this function mostly initialized/compute the preconditioner. In the future
    * we might, for instance, implement column reodering for faster matrix vector products.
    *
    * \warning this class stores a reference to the matrix A as well as some
    * precomputed values that depend on it. Therefore, if \a A is changed
    * this class becomes invalid. Call compute() to update it with the new
    * matrix A, or modify a copy of A.
    */
  Derived& compute(const MatrixType& A)
  {
    mp_matrix = &A;
    m_preconditioner.compute(A);
    m_isInitialized = true;
    m_info = Success;
    return derived();
  }

  /** \internal */
  Index rows() const { return mp_matrix->rows(); }
  /** \internal */
  Index cols() const { return mp_matrix->cols(); }

  /** \returns the tolerance threshold used by the stopping criteria */
  RealScalar tolerance() const { return m_tolerance; }
  
  /** Sets the tolerance threshold used by the stopping criteria */
  Derived& setTolerance(RealScalar tolerance)
  {
    m_tolerance = tolerance;
    return derived();
  }

  /** \returns a read-write reference to the preconditioner for custom configuration. */
  Preconditioner& preconditioner() { return m_preconditioner; }
  
  /** \returns a read-only reference to the preconditioner. */
  const Preconditioner& preconditioner() const { return m_preconditioner; }

  /** \returns the max number of iterations */
  int maxIterations() const { return m_maxIterations; }
  
  /** Sets the max number of iterations */
  Derived& setMaxIterations(int maxIters)
  {
    m_maxIterations = maxIters;
    return derived();
  }

  /** \returns the number of iterations performed during the last solve */
  int iterations() const
  {
    eigen_assert(m_isInitialized && "ConjugateGradient is not initialized.");
    return m_iterations;
  }

  /** \returns the tolerance error reached during the last solve */
  RealScalar error() const
  {
    eigen_assert(m_isInitialized && "ConjugateGradient is not initialized.");
    return m_error;
  }

  /** \returns the solution x of \f$ A x = b \f$ using the current decomposition of A.
    *
    * \sa compute()
    */
  template<typename Rhs> inline const internal::solve_retval<Derived, Rhs>
  solve(const MatrixBase<Rhs>& b) const
  {
    eigen_assert(m_isInitialized && "IterativeSolverBase is not initialized.");
    eigen_assert(rows()==b.rows()
              && "IterativeSolverBase::solve(): invalid number of rows of the right hand side matrix b");
    return internal::solve_retval<Derived, Rhs>(derived(), b.derived());
  }
  
  /** \returns the solution x of \f$ A x = b \f$ using the current decomposition of A.
    *
    * \sa compute()
    */
  template<typename Rhs>
  inline const internal::sparse_solve_retval<IterativeSolverBase, Rhs>
  solve(const SparseMatrixBase<Rhs>& b) const
  {
    eigen_assert(m_isInitialized && "IterativeSolverBase is not initialized.");
    eigen_assert(rows()==b.rows()
              && "IterativeSolverBase::solve(): invalid number of rows of the right hand side matrix b");
    return internal::sparse_solve_retval<IterativeSolverBase, Rhs>(*this, b.derived());
  }

  /** \returns Success if the iterations converged, and NoConvergence otherwise. */
  ComputationInfo info() const
  {
    eigen_assert(m_isInitialized && "IterativeSolverBase is not initialized.");
    return m_info;
  }
  
  /** \internal */
  template<typename Rhs, typename DestScalar, int DestOptions, typename DestIndex>
  void _solve_sparse(const Rhs& b, SparseMatrix<DestScalar,DestOptions,DestIndex> &dest) const
  {
    eigen_assert(rows()==b.rows());
    
    int rhsCols = b.cols();
    int size = b.rows();
    Eigen::Matrix<DestScalar,Dynamic,1> tb(size);
    Eigen::Matrix<DestScalar,Dynamic,1> tx(size);
    for(int k=0; k<rhsCols; ++k)
    {
      tb = b.col(k);
      tx = derived().solve(tb);
      dest.col(k) = tx.sparseView(0);
    }
  }

protected:
  void init()
  {
    m_isInitialized = false;
    m_maxIterations = 1000;
    m_tolerance = NumTraits<Scalar>::epsilon();
  }
  const MatrixType* mp_matrix;
  Preconditioner m_preconditioner;

  int m_maxIterations;
  RealScalar m_tolerance;
  
  mutable RealScalar m_error;
  mutable int m_iterations;
  mutable ComputationInfo m_info;
  mutable bool m_isInitialized;
};

namespace internal {
 
template<typename Derived, typename Rhs>
struct sparse_solve_retval<IterativeSolverBase<Derived>, Rhs>
  : sparse_solve_retval_base<IterativeSolverBase<Derived>, Rhs>
{
  typedef IterativeSolverBase<Derived> Dec;
  EIGEN_MAKE_SPARSE_SOLVE_HELPERS(Dec,Rhs)

  template<typename Dest> void evalTo(Dest& dst) const
  {
    dec().derived()._solve_sparse(rhs(),dst);
  }
};

}

#endif // EIGEN_ITERATIVE_SOLVER_BASE_H

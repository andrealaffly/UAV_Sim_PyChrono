import numpy as np
from dataclasses import dataclass

class ProjectionOperator:
  """
    Implementation of the projection operator to be used in the MRAC controllers.
    For reference: A. L'Afflitto, "Notes on Adaptive Control and Estimation", Springer, Sec. 3.5. or
    E. Lavretsky, K. Wise, "Robust and Adaptive Control", Springer 2013, Sec. 11.4
  """

  class Types:
    @dataclass
    class ConvexFunctionOutput:
      """
      Holds the output of the convex function.

      Attributes:
      -----------
      h_function : float
        Scalar value of the convex set function.

      dh_dx_jacobian : np.ndarray
        Row vector (1 x N) representing the Jacobian (derivative of h with respect to x).
      """
      h_function: float
      dh_dx_jacobian: np.ndarray

  @staticmethod
  def generateMatrixFromDiagonal(S_diagonal: np.ndarray) -> np.ndarray:
    """
    Generate a square diagonal matrix S from the input column vector S_diagonal.

    Parameters:
    -----------
    S_diagonal : np.ndarray
      1D numpy array containing the diagonal elements.

    Returns:
    --------
    S : np.ndarray
      Square diagonal matrix with S_diagonal on the diagonal.
    """
    assert S_diagonal.ndim == 1, "Input must be a 1D array (column vector)"

    S = np.zeros((S_diagonal.size, S_diagonal.size), dtype=S_diagonal.dtype)
    np.fill_diagonal(S, S_diagonal)
    return S
  
  @staticmethod
  def generateEllipsoidMatrixFromDiagonal(S_diagonal: np.ndarray) -> np.ndarray:
    """
    Generate a matrix S for an ellipsoid from its semi-axis length terms contained in S_diagonal.
    If S = diag(1/a^2, 1/b^2, 1/c^2), then the semi-axis lengths are [a, b, c].

    Parameters:
    -----------
    S_diagonal : np.ndarray
      1D numpy array containing the semi-axis lengths of the ellipsoid.

    Returns:
    --------
    S : np.ndarray
      Square diagonal matrix with 1/(S_diagonal^2) on the diagonal.
    """
    assert S_diagonal.ndim == 1, "Input must be a 1D array (column vector)"

    S = np.zeros((S_diagonal.size, S_diagonal.size), dtype=S_diagonal.dtype)
    np.fill_diagonal(S, 1.0 / np.power(S_diagonal, 2))
    return S
  
  @staticmethod
  def computeEpsilonFromAlpha(alpha: float) -> float:
    """
    Compute the epsilon parameter from the scaling coefficient alpha.

    Parameters:
    -----------
    alpha : float
      Scaling coefficient.

    Returns:
    --------
    epsilon : float
      Computed epsilon value.
    """
    epsilon = (1.0 / (alpha * alpha)) - 1.0
    return epsilon
  
  class Ellipsoid:
    @staticmethod
    def convexFunction(
      x: np.ndarray,
      x_e: np.ndarray,
      S: np.ndarray,
      epsilon: float
      ) -> 'ProjectionOperator.Types.ConvexFunctionOutput':
      """
      Convex function to compute h and dh_dx. 
      The outer convex set is an ellipsoid centered in x_e with semi-axis length described by the diagonal elements
      that populate the S matrix. If S = diag(1/a^2, 1/b^2, 1/c^2) then the semi-axis lengths are [a, b, c].
      The inner convex set is the outer convex set ellipsoid scaled by the coefficient alpha = 1/sqrt(1 + epsilon).

      Parameters:
      -----------
      x : np.ndarray
        Current point (N x 1).

      x_e : np.ndarray
        Ellipsoid center (N x 1).

      S : np.ndarray
        Ellipsoid shape matrix (N x N).

      epsilon : float
        Scaling parameter.

      Returns:
      --------
      ConvexFunctionOutput
        Contains h_function and dh_dx_jacobian.
      """
      x_diff = x - x_e
      quadratic_term = float(x_diff.T @ S @ x_diff)
      h_function = ((1.0 + epsilon) * quadratic_term - 1.0) / epsilon
      dh_dx_jacobian = (2.0 * (1.0 + epsilon) / epsilon) * (x_diff.T @ S)
      return ProjectionOperator.Types.ConvexFunctionOutput(h_function, dh_dx_jacobian)
    
    @staticmethod
    def projectionVector(
      x: np.ndarray,
      x_d: np.ndarray,
      x_e: np.ndarray,
      S: np.ndarray,
      epsilon: float
      ) -> tuple[np.ndarray, bool]:
      """
      Project the vector x_d based on the ellipsoid convex function.

      Parameters:
      -----------
      x : np.ndarray
        Current state vector (N x 1).

      x_d : np.ndarray
        Derivative of x wrt time (N x 1).

      x_e : np.ndarray
        Ellipsoid center vector (N x 1).

      S : np.ndarray
        Ellipsoid shape matrix (N x N).

      epsilon : float
        Scaling parameter.

      Returns:
      --------
      VectorProjectionOutput
        The projected vector and a flag indicating if projection was activated.
      """
      cfo = ProjectionOperator.Ellipsoid.convexFunction(x, x_e, S, epsilon)
      h_function = cfo.h_function
      dh_dx_jacobian = cfo.dh_dx_jacobian  # Shape (1, N)

      projection_operator_activated = (h_function > 0 and (dh_dx_jacobian @ x_d) > 0)

      if projection_operator_activated:
        dh_dx_jacobian_T = dh_dx_jacobian.T  # Shape (N, 1)
        numerator = h_function * dh_dx_jacobian_T * (dh_dx_jacobian @ x_d)
        denominator = (dh_dx_jacobian @ dh_dx_jacobian_T)[0, 0]
        x_d_modified = x_d - numerator / denominator
        return (x_d_modified, True)
      else:
        return (x_d, False)
      
    @staticmethod
    def projectionMatrix(
      matrix: np.ndarray,
      matrix_d: np.ndarray,
      x_e: np.ndarray,
      S: np.ndarray,
      epsilon: float
      ) -> tuple[np.ndarray, bool]:
      """
      Project a matrix by flattening it into a vector, applying vector projection, 
      then reshaping back to the original matrix shape.

      Parameters:
      - matrix: current state matrix.
      - matrix_d: time derivative of the state matrix, same shape as matrix.
      - x_e: ellipsoid center vector matching flattened matrix size.
      - S: ellipsoid shape matrix matching flattened size.
      - epsilon: scaling parameter for projection.

      Returns:
      - MatrixProjectionOutput with the projected matrix and activation flag.

      Note:
      - Uses column-major order (Fortran) for reshaping to align with Eigen layout.
      """
      assert matrix.shape == matrix_d.shape, "Input matrices must have the same shape"

      original_shape = matrix.shape
      total_elements = matrix.size

      # Reshape matrices into vectors (column-major / Fortran order consistent with Eigen)
      reshaped_matrix = matrix.reshape((total_elements, 1), order='F')
      reshaped_matrix_d = matrix_d.reshape((total_elements, 1), order='F')

      # Apply projectionVector to the reshaped vectors
      (projected_vector, activated) = ProjectionOperator.Ellipsoid.projectionVector(
        reshaped_matrix, reshaped_matrix_d, x_e, S, epsilon
      )

      # Reshape the projected vector back to original matrix shape (column-major)
      projected_matrix = projected_vector.reshape(original_shape, order='F')

      return (projected_matrix, activated)
  

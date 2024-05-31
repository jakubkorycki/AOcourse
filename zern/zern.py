import numpy as np
import matplotlib.pyplot as plt

def nm2noll(n, m):
        """Convert indices `(n, m)` to the Noll's index `k`.

        Noll's index `k` starts from one and Python indexing is
        zero-based.

        """
        k = n * (n + 1) // 2 + abs(m)
        if (m <= 0 and n % 4 in (0, 1)) or (m >= 0 and n % 4 in (2, 3)):
            k += 1
        return k

def noll2nm(j):
    """
    Convert Noll index to (n, m) indices for Zernike polynomials.
   
    Parameters:
    j (int): Noll index (1-based).
   
    Returns:
    tuple: (n, m) where n is the radial order and m is the azimuthal order.
    """
    if j < 1:
        raise ValueError("Noll index must be a positive integer.")

    n = 0
    while (n * (n + 1)) // 2 < j:
        n += 1

    n -= 1
    k = j - (n * (n + 1)) // 2 - 1
    m = -n + 2 * (k // 2)

    if k % 2 == 1:
        m = -m

    return n, m

def cartesian_to_polar(x, y):
    """
    Convert Cartesian coordinates to polar coordinates.
    
    Parameters:
    x (array-like): x-coordinate.
    y (array-like): y-coordinate.
    
    Returns:
    tuple: (rho, phi) where rho is the radial coordinate and phi is the azimuthal coordinate.
    """
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, phi

def polar_to_cartesian(rho, phi):
    """
    Convert polar coordinates to Cartesian coordinates.
    
    Parameters:
    rho (array-like): Radial coordinate.
    phi (array-like): Azimuthal coordinate.
    
    Returns:
    tuple: (x, y) where x and y are the Cartesian coordinates.
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y

def radial_zernike(n, m, rho):
    """
    Calculate the radial part of the Zernike polynomial.
    
    Parameters:
    n (int): Non-negative integer.
    m (int): Non-negative integer, m <= n and (n - m) is even.
    rho (array-like): Radial coordinate.
    
    Returns:
    numpy.ndarray: Radial part of Zernike polynomial evaluated at rho.
    """
    if (n - m) % 2 != 0:
        raise ValueError("n - m must be even.")
    
    Rnm = np.zeros_like(rho)
    for k in range((n - m) // 2 + 1):
        Rnm += (-1)**k * np.math.factorial(n - k) / (
            np.math.factorial(k) * 
            np.math.factorial((n + m) // 2 - k) * 
            np.math.factorial((n - m) // 2 - k)
        ) * rho**(n - 2*k)
    
    return Rnm

def zernike_polynomial(n, m, rho, phi):
    """
    Calculate the Zernike polynomial.
    
    Parameters:
    n (int): Non-negative integer.
    m (int): Integer such that |m| <= n and (n - m) is even.
    rho (array-like): Radial coordinate.
    phi (array-like): Azimuthal coordinate.
    
    Returns:
    numpy.ndarray: Zernike polynomial evaluated at (rho, phi).
    """
    Rnm = radial_zernike(n, abs(m), rho)
    
    if m >= 0:
        return Rnm * np.cos(m * phi)
    else:
        return Rnm * np.sin(abs(m) * phi)

def gradient_zernike(n, m, rho, phi):
    """
    Calculate the gradient of the Zernike polynomial.
    
    Parameters:
    n (int): Non-negative integer.
    m (int): Integer such that |m| <= n and (n - m) is even.
    rho (array-like): Radial coordinate.
    phi (array-like): Azimuthal coordinate.
    
    Returns:
    tuple of numpy.ndarray: Gradients of Zernike polynomial with respect to rho and phi.
    """
    Rnm = radial_zernike(n, abs(m), rho)
    dRnm_drho = np.gradient(Rnm, rho, edge_order=2)
    
    if m >= 0:
        dZ_drho = dRnm_drho * np.cos(m * phi)
        dZ_dphi = -m * Rnm * np.sin(m * phi)
    else:
        dZ_drho = dRnm_drho * np.sin(abs(m) * phi)
        dZ_dphi = abs(m) * Rnm * np.cos(abs(m) * phi)
    
    return dZ_drho, dZ_dphi

def zernike_cartesian(n, m, x, y):
    """
    Calculate the Zernike polynomial in Cartesian coordinates.
    
    Parameters:
    n (int): Non-negative integer.
    m (int): Integer such that |m| <= n and (n - m) is even.
    x (array-like): x-coordinate.
    y (array-like): y-coordinate.
    
    Returns:
    numpy.ndarray: Zernike polynomial evaluated at (x, y).
    """
    rho, phi = cartesian_to_polar(x, y)
    return zernike_polynomial(n, m, rho, phi)

def gradient_zernike_cartesian(n, m, x, y):
    """
    Calculate the gradient of the Zernike polynomial in Cartesian coordinates.
    
    Parameters:
    n (int): Non-negative integer.
    m (int): Integer such that |m| <= n and (n - m) is even.
    x (array-like): x-coordinate.
    y (array-like): y-coordinate.
    
    Returns:
    tuple of numpy.ndarray: Gradients of Zernike polynomial with respect to x and y.
    """
    rho, phi = cartesian_to_polar(x, y)
    dZ_drho, dZ_dphi = gradient_zernike(n, m, rho, phi)
    
    dZ_dx = dZ_drho * np.cos(phi) - dZ_dphi * np.sin(phi) / rho
    dZ_dy = dZ_drho * np.sin(phi) + dZ_dphi * np.cos(phi) / rho
    
    return dZ_dx, dZ_dy


if __name__ == "__main__":
    # Example usage
    x = np.linspace(-1, 1, 100)
    y = np.linspace(-1, 1, 100)
    X, Y = np.meshgrid(x, y)
    
    n, m = 3, 3
    print(cartesian_to_polar(1, 1))
#    Z_cartesian = zernike_cartesian(n, m, X, Y)
#    plt.imshow(Z_cartesian)
#    dZ_dx_cartesian, dZ_dy_cartesian = gradient_zernike_cartesian(n, m, X, Y)

Thread-safe geosearch engine with SIMD optimization (no Mercator projection, just plain Eucledian distances). 
1) Detects if a point with radius intersects with geometries in index.
2) Vectorised Ramer–Douglas–Peucker algorithm
3) Some lock free technichs undehood (write-preferring readers–writer shared lock)
4) Optional: exchange data with Postgresql by triggering events

### Install Eigen
Eigen is header only, this will install it in system include PATH
Download Eigen from http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
Go to Eigen top directory

```
  mkdir build
  cd build
  cmake ..
  sudo make install
```

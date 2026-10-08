# state_lib

Small utility library for defining states as a vector and matrix.

There are two use cases for this library.

- Holding a dictionary of state variables with compile-time optimizations for accessing and mutating members.
- Tracking a set of $n$ state variables alongside an $n \times n$ covariance matrix.

They are not exclusive, and the second case builds on the functionality of the first. The library provides state vector and state variable classes to fit either mould. 

# Rationale

By using state vectors you avoid the use of magic numbers. Typically, when modelling physics you as a programmer assign meaning to the numbers used to introspect algebraic types like vectors or matrices. This approach is needlessly error-prone, and we would like the programmer to make their intentions bare when retrieving yaw from a vector with the number two (as an example). This library provides as much flexibility as possible to avoid the user's temptation to fall back on magic numbers. Given a vector, we provide four options for accessing one or multiple members in two languages

| Language          | Syntax              | Supported Operations       | Explanation                                  |
| :---------------- | :------:            | -----                      | ----                                         |
| C++               |   ```vec(pose)```   | Access / Mutate            | operator() overload                          |
| C++               |   ```vec[pose]```   | Access singletons / Mutate | operator[] overload and masks for assignment |
| Python            |  ```vec.pose```     | Access / Mutate            | Bindings for both operators                  |
| Python            |  ```vec[pose]```    | Access / Mutate            | Bindings for both operators                  |

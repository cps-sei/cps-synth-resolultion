# STL Enforcer

The following is the list of files that are related to the STL-based enforcer:

Signal.(h | cpp): Stores a signal as a timed sequence of vectors, where each vector is a mapping from a name (e.g., "pos_east_m") to a value.

SigFun.(h | cpp): A function from a signal to some value (e.g., f(signal, t) = TTI at time "t"). Used as part of the atomic expression in STL. This is a base class should be extended by application-specific sginal functions.

TTIFun.(h | cpp): Extension of SigFun that computes time-to-intercept (TTI).

StlExpr.(h | cpp): Used to construct different types of STL expressions (conjunction, negation, implies, globally, past globally).

StlEnforcer.(h | cpp): An enforcer that monitors STL properties at each time "tick" and perform corrective actions when they are violated.







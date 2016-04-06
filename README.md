# Path-Planning-Replication

This is a replication I performed of the article "A Statiscally Rigorous Comparison of 2D Path Planning Algorithms".

##Testing Environment
|Operating System|CPU|RAM|IDE|
|:---:|:---:|:---:|:---:|
|Windows 10 Pro 64-Bit|Intel i7 4710HQ|8GB DDR3 1600Mhz|Visual Studio Community 2015|

##Results of Replication
The Results of this replication are found in the root folder of this repository "Results.xlsx".

##DOI for above article
DOI: http://dx.doi.org/10.1093/comjnl/bxu137

##REQUIREMENTS
- C# 6.0
- Math.Net Library (For statistical features)
- CPU with Multiple Cores (Map Traversal is Threaded to Save Computation Time)
- At Least 4GB of Ram (Maps take up a lot of resources. Ex: 500, 500x500 Maps with 5x5 Obstacles Takes ~600MB Ram). Also there are some memory leaks in the program, and I have not got around to fixing them yet.

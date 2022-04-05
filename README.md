# Indp
[InDP: In-Network Data Processing for Wireless Sensor Networks](https://ieeexplore.ieee.org/abstract/document/8824851)

InDP is a in-network data procesing protocol which design and implement Principal Component Analysis on the edge. InDP communication component is based on [Codecast](https://ieeexplore.ieee.org/abstract/document/8480037).

In InDP, you may control the number of nodes acting as source nodes during running time as nodes use control information to share if they have data to share or not!

## Running an example on Cooja
You may directly open the given simulation file on COOJA

["30_sources_demo"](https://github.com/ebramkw/indp/blob/main/contiki/apps/indp-test/30_sources_demo.csc).
In this example, 30 nodes are collecting sensors data, computing local PCA each on its local data, share their local PCA result in a many-to-many fashion, and finally compute the global PCA which can be used for data comopression or outlier detection.

## Running on a testbed
You may update the nodes settings under [deploy](https://github.com/ebramkw/indp/tree/main/contiki/core/deploy).
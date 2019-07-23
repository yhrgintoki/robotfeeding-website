---
title: "Basic Autonomous Navigation"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
duration: 15
difficulty: Intermediate
summary: This tutorial will teach you how to initialize and operate MuSHRs out-of-the-box autonomous navigation stack.
weight: 2
---

## Requirements

If you intend to run this tutorial on the real car,  complete the Mapping Tutorial first. We will also assume you have built your car with the LiDaR.

## Navigation Stack Overview

At the highest level MuSHR's navigation stack consists of two principal components:

1. Receding Horizon Controller Node 
2. Localization Node

The Receding Horizion Control (RHC) node is responsible for planning the motions and generating controls for the car. The implementation we ship with the car uses Model Predictive Control (MPC) to generate control signals which are sent to the car's motor controller (VESC). In order for the controller to know whether it is in the proximity of obstacles, it must know its location on a known map. Solving this problem is called "localization". The Localization Node is implemented using a method called Particle Filtering which in this case relies primarily on a data stream from the laser scanner.

This tutorial does not cover Model Predictive Control and Particle Filtering in depth. However recommend this tutoral to learn more about MPC and this one for Particle Filtering.

## Launching the Navigation Stack

Launch simultaneously, on the robot: 

{{< highlight bash >}}
$ roslaunch mushr_pf localize.launch
$ roslaunch mushr_rhc tl.launch
$ cd ..
{{< / highlight >}}

Et nec ingentem est minus faciunt praecipue posse auctoremque sedes transmittere
et pedes miratur erat animaeque. Tellus admonuit humanam funes, sagittis et
licet! Inserui quamvis Clymeni.

- Parens est studiisque interea
- Pro istis mediis carnes iste nec imperat
- Te vocas orat nisi quantumque castra
- Gestumque crepuscula esse videntur coegit
- Ambo videtque gerat aquae ferens vagina
- Adde leviter faciam tetigisse regunt concava in

Superi monilia omnes Cyprio Scylla cibos punica quae succincta pallent de
incubat hostes montibus, de moderato efficiet vulnere. Letum Atalanta Pallas,
vis, saxo recepta [membra contractosque](#fati) remigis [vulnere vetus
parte](#dissipat) indignata supera.

Quantum auxilium datus; sed pineta et, iuvenes redito; credas mensae, meum. Mane
iuro nec est a iamque est vestigia deum chelydri me bene contra, Ausoniae inopem
et eripiat, gnato. Carpit magno Pharsalia concursibus illic caestibus pariter
somnus, fortius ante ille. Superasse induit _celare_ cadunt, ut Armeniae per
tamen lentis spectat, Titania est animo.

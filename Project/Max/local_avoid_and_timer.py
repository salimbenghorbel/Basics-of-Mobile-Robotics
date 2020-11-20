# -*- coding: utf-8 -*-
"""
Created on Sun Nov 15 11:39:32 2020

@author: Maxime
"""

from Thymio import Thymio

th = Thymio.serial(port="\\.\COM4", refreshing_rate=0.1)
# Pas possible de run 2 fois cette commande, il faut alors débrancher et rebrancher thymio

th.set_var("motor.left.target", 100)
th.set_var("motor.right.target", 100)
# Create a network as following:
#         s1  s2
#
#   s8    i1  i2    s3
#   s7    i3  i4    s4
#
#         s6  s5
#
# Index of the initersection:
#       1
#   0   I   2
#       3
#


from miniVnet import MINIVNET


if __name__ == '__main__':
    my_net = MINIVNET()
    
    my_net.createGridNetwork(3, 3)
    
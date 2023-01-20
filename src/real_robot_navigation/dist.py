
dist = [651.6302,  325.8151,  217.2101,  162.9075,
        130.3260,  108.6050,   93.0900,   81.4538,   72.4034]
dist = [15.33247435294118e2,    4.572843228070175e2,    2.687134680412372e2,    1.902569810218978e2,
        1.472610531073447e2,    1.201161585253456e2,   1.014210365758755e2,  87.761637710437700,    77.344826112759650]
dist = [521.6287,  410.0752,  368.7916,  347.2846,
        334.0895,  325.1681,  318.7336,  313.8732,  279.0000]
dist_o = [554.5799,  299.6004,  205.2380,  156.0792,
          125.9189,  105.5272,   90.8196,   79.7101,    0.0000]
dist = [521.6287,  410.0752,  368.7916,  347.2846,
        334.0895,  325.1681,  318.7336,  313.8732,  310.0723]
dista = [-157.7301,   43.0652,  117.3753,  156.0878,
         179.8389,  195.8974,  207.4796,  216.2283,  223.0699]
distb = [-32.9500,  110.4751,  163.5538,  191.2056,
         208.1707,  219.6410,  227.9140,  234.1631,  239.0499]
dist_a = [22.5077,  140.4351,  184.0776,  206.8135,  220.7625,
          230.1937,  236.9960,  242.1341,  246.1522]  # smaller
dist_b = [-102.2723,   73.0251,  137.8991,  171.6957,
          192.4308,  206.4501,  216.5616,  224.1993,  230.1721]

dists = []
dists.append(dist_b)
dists.append(dist_a)
a_s = []
for dist in dists:
    val = 5
    clo = 0
    mid = 2
    fa = 7
    validate = dist[val]
    far = dist[fa]
    middle = dist[mid]
    close = dist[clo]

    offset = 1
    offset = 1-0.575
    offset = 1+0.175
    y_validate = val+offset
    y_far = fa+offset
    y_middle = mid+offset
    y_close = clo+offset

    y_validate = 1/y_validate
    y_far = 1/y_far
    y_middle = 1/y_middle
    y_close = 1/y_close

    # print(far/y_far)
    # print(middle/y_middle)
    # print(close/y_close)

    a = (far-close)/(y_far-y_close)
    b = far - a*y_far
    
#     a = ((399-298)-(424-298))/((1/2.55)-(1/2.99))
#     b = (600-399) - a*(1/2.55)

    print('b is :', b)
    print('a is:', a)

    print(a*y_validate+b-validate)
    print(a*y_far+b-far)
    print(a*y_middle+b-middle)
    print(a*y_close+b-close)
    a_s.append(a)

print(a_s[0]/a_s[1])
print((98-43)/(80-43))

"""
tensor(2.98769) tensor(398.) use this for cal
tensor(2.99927) tensor(399.) use this for cal

tensor(2.52879) tensor(424.) use this for cal
tensor(2.55383) tensor(424.) use this for cal

"""
def compute_sigma_pt(prior):
    return prior + 9

def compute_sigma_t(prior):
    return (prior * 100) / (prior + 100)

prior = 1814.87
t = 4
while True:
    sigma_pt = compute_sigma_pt(prior)
    sigma_t = compute_sigma_t(sigma_pt)
    t+=1

    if sigma_t == prior:
        break
    else:
        prior = sigma_t
    
print(t)
print(sigma_t)



Instead of dividing by dt just so I can add accel should I instead just swap vel tbh?



v_1f = [v_1 (m_1 - m_2) + 2 m_2 v_2]/(m_1 + m_2)
as m2 goes to infinty since a wall with infinte mass
m_1 - m_2 = -m_2
m_1 + m_2 = m_2
v_2 = 0
v_1f = -v_1

so the change in velocity is -2v_1 since v_1f - v_1 = -v_1 - v_1 = -2v_1


       
// Project v in the direction of the normal from wall
// projection is
// v_normal = [(v . n) / (n . n)] * n / n. n 
// v_normal = [(v . n) / |n|] * n / |n|
// v_normal = [|v| |n| cos(theta) / |n|] * n / |n|
// v_normal = |v| cos(theta) * n / |n|
// v_normal = v cos(theta) * n_unit
// Makes sense cause we get v cos of angle whic his projected onto the normal vector
// Then we multiple by normal vector to vector distance
// So easy way to show in code is prob v_normal = v . n * |n|^2


False Sharing
Matrix Mult
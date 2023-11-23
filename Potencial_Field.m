function [linear_x_list, linear_z_list, U_list, nr_list, Vr_list] = Potencial_Field(xo, x0, xs)
    da = 1;
    dr = 1;
    a = 1;
    Va = [0; 0; 0];
    Vr = [0; 0; 0];
    U = [0; 0];
    linear_x_list = [];
    linear_z_list = [];
    U_list = [];
    nr_list = [];
    Vr_list = [];

    nr = norm(xo(1:2) - x0(1:2));
    nr_list = [nr_list, nr]; % Storing nr
    
    Vr(1:2) = a * (1 / nr - dr / (nr * nr)) * (xo(1:2) - x0(1:2)) / nr;
    Vr_list = [Vr_list, Vr]; % Storing Vr
    U = Vr*nr;
    U_list = [U_list, U]; % Storing U
    linear_x = U(1)/2;
    linear_x_list = [linear_x_list, linear_x]; % Storing linear_x
    linear_z = U(2)/2;
    linear_z_list = [linear_z_list, linear_z]; % Storing linear_z
end
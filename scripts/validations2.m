wTa=[  1.000    0.000    0.000 1.03500000
   0.000    1.000    0.000 0.04500000
   0.000    0.000    1.000 1.16500000
   0.000    0.000    0.000    1.000]

wTb=[0.994   -0.061   -0.093 0.44358785
   0.073    0.988    0.136 0.36953497
   0.084   -0.142    0.986 0.48466255
   0.000    0.000    0.000    1.000]
   
P2=wTa*(wTb)^-1   
P2(1:3,4)-[ 1.03500000;0.045;1.165];

wTa^-1*(wTb)

wTa(1:3,4)-wTb(1:3,4)


inv(T)=R' -R

%------------------------------------------------
% Old_method=wTa*(wTb)^-1 yeah it makes no sense whatsoever
% R=wRa*(bRw) -Diffence between the two rotations
% P=wRa bPw 
%正运动学
function T06 = scara_fkine(d1,theta2,theta3,theta4,theta5,theta6,DH_link)
    DH=[0   DH_link.d(1)+d1       DH_link.a(1)    DH_link.alpha(1);
     theta2    DH_link.d(2)       DH_link.a(2)    DH_link.alpha(2);
     theta3    DH_link.d(3)       DH_link.a(3)    DH_link.alpha(3);
     theta4    DH_link.d(4)       DH_link.a(4)    DH_link.alpha(4);
     theta5    DH_link.d(5)       DH_link.a(5)    DH_link.alpha(5);
     theta6    DH_link.d(6)       DH_link.a(6)    DH_link.alpha(6);
   ];
     T01=[1                            0                              0              DH_link.base(1);
         0                            1                             0               DH_link.base(2);
         0                            0                              1              DH(1,2)+DH_link.base(3);
         0                            0                              0               1;
    ];
    T12=[cos(DH(2,1))                 -sin(DH(2,1))                  0               DH(2,3)*cos(DH(2,1));
         sin(DH(2,1))                 cos(DH(2,1))                   0               DH(2,3)*sin(DH(2,1));
         0                            0                              1               0;
         0                            0                              0               1;
    ]  
     T23=[cos(DH(3,1))                 -sin(DH(3,1))                 0               DH(3,3)*cos(DH(3,1));
         sin(DH(3,1))                 cos(DH(3,1))                   0               DH(3,3)*sin(DH(3,1));
         0                            0                              1               0;
         0                            0                              0               1;
    ]
     T34=[cos(DH(4,1))              0            -sin(DH(4,1))                  0;
          sin(DH(4,1))              0            cos(DH(4,1))                   0;
          0                         -1           0                              DH(4,2);
          0                         0            0                              1
      ];
     T45=[cos(DH(5,1))                  0                    sin(DH(5,1))               0;
         sin(DH(5,1))                   0                    -cos(DH(5,1))              0;
         0                              1                     0                         0;
         0                              0                     0                         1
         ];

      T56=[cos(DH(6,1))                 -sin(DH(6,1))                    0               0;
           sin(DH(6,1))                 cos(DH(6,1))                    0               0;
           0                              0                             1               100;
           0                              0                             0               1
           ];
    T06=T01*T12*T23*T34*T45*T56; 
end
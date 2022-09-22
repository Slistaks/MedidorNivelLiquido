
%cryo apagado, frecuencia de muestreo 400S/s
%mm indicados es nivel respecto al inicio del ambito de medida

vacio=[-6.534,-6.531,-6.535,-6.531,-6.533,-6.540,-6.532,-6.530,-6.530,-6.537,-6.535,-6.523,-6.543,-6.537,-6.533,-6.531,-6.528,-6.531,-6.537,-6.533,-6.544,-6.539,-6.522,-6.526,-6.543,-6.521,-6.532,-6.544,-6.532,-6.530,-6.534,-6.543,-6.539,-6.524,-6.524,-6.535,-6.528,-6.522,-6.548,-6.534,-6.526,-6.540,-6.527,-6.531,-6.534,-6.535,-6.539,-6.524,-6.536,-6.534,-6.542,-6.530,-6.533,-6.532,-6.533,-6.538,-6.538,-6.532,-6.526,-6.519,-6.521,-6.541,-6.530,-6.559,-6.546,-6.545,-6.537,-6.529,-6.532,-6.535,-6.517,-6.539,-6.529,-6.530,-6.538,-6.530,-6.534,-6.533,-6.541,-6.513,-6.543,-6.529,-6.537,-6.523,-6.535,-6.527,-6.543,-6.524,-6.538,-6.525,-6.527,-6.531,-6.539,-6.548,-6.543,-6.533,-6.537,-6.534,-6.537,-6.544];
vacio=mean(vacio);

nivel_25mm=[-4.285,-4.285,-4.282,-4.296,-4.286,-4.269,-4.287,-4.280,-4.279,-4.283,-4.284,-4.284,-4.289,-4.284,-4.267,-4.292,-4.267,-4.278,-4.278,-4.282,-4.281,-4.277,-4.278,-4.277,-4.278,-4.281,-4.290,-4.276,-4.295,-4.268,-4.280,-4.277,-4.278,-4.287,-4.290,-4.295,-4.261,-4.263,-4.279,-4.279,-4.281,-4.285,-4.278,-4.290,-4.266,-4.288,-4.293,-4.280,-4.276,-4.295,-4.275,-4.277,-4.274,-4.296,-4.278,-4.282,-4.278,-4.281,-4.292,-4.275,-4.275,-4.289,-4.275,-4.281,-4.280,-4.282,-4.277,-4.273,-4.283,-4.271,-4.283,-4.274,-4.271,-4.299,-4.259,-4.259,-4.282,-4.288,-4.287,-4.278,-4.280,-4.290,-4.282,-4.280,-4.290,-4.283,-4.279,-4.287,-4.286,-4.293,-4.271,-4.268,-4.295,-4.275,-4.280,-4.291,-4.276,-4.297,-4.289,-4.258];
nivel_25mm=mean(nivel_25mm);

nivel_35mm=[-3.646,-3.633,-3.632,-3.656,-3.641,-3.638,-3.634,-3.631,-3.630,-3.643,-3.645,-3.635,-3.620,-3.618,-3.640,-3.626,-3.629,-3.637,-3.641,-3.647,-3.598,-3.657,-3.637,-3.633,-3.664,-3.635,-3.619,-3.668,-3.643,-3.646,-3.638,-3.622,-3.632,-3.658,-3.631,-3.666,-3.622,-3.589,-3.672,-3.642,-3.647,-3.638,-3.648,-3.652,-3.650,-3.622,-3.648,-3.643,-3.633,-3.650,-3.637,-3.647,-3.658,-3.639,-3.636,-3.665,-3.671,-3.625,-3.617,-3.636,-3.648,-3.643,-3.642,-3.625,-3.652,-3.641,-3.644,-3.637,-3.637,-3.647,-3.648,-3.639,-3.653,-3.638,-3.648,-3.621,-3.658,-3.650,-3.638,-3.648,-3.659,-3.621,-3.679,-3.636,-3.641,-3.640,-3.656,-3.647,-3.643,-3.642,-3.636,-3.654,-3.668,-3.642,-3.643,-3.633,-3.641,-3.630,-3.647,-3.638];
nivel_35mm=mean(nivel_35mm);

nivel_45mm=[-2.976,-2.957,-2.990,-2.970,-2.996,-2.959,-2.992,-2.951,-2.971,-2.981,-2.985,-2.989,-2.968,-2.977,-2.973,-2.984,-2.989,-2.969,-2.969,-2.989,-2.987,-2.990,-2.974,-2.984,-2.978,-2.999,-2.984,-2.969,-2.969,-2.979,-2.983,-2.971,-2.959,-2.971,-2.979,-2.999,-2.956,-2.986,-2.980,-2.998,-2.988,-3.006,-2.996,-2.989,-3.007,-2.989,-3.001,-2.992,-2.993,-2.991,-3.030,-3.014,-2.963,-2.975,-2.984,-2.977,-2.987,-2.980,-2.993,-2.962,-2.973,-3.007,-2.969,-2.984,-2.973,-3.010,-2.984,-2.994,-2.963,-2.972,-2.988,-2.989,-2.983,-2.990,-2.983,-2.965,-2.986,-2.990,-3.000,-2.985,-2.989,-2.977,-2.989,-2.980,-3.000,-2.985,-2.977,-2.983,-2.995,-3.002,-2.992,-2.980,-2.973,-2.963,-2.983,-2.985,-2.986,-2.986,-2.983,-2.984];
nivel_45mm=mean(nivel_45mm);

nivel_55mm=[-2.304,-2.299,-2.306,-2.310,-2.300,-2.333,-2.322,-2.293,-2.283,-2.299,-2.309,-2.295,-2.298,-2.353,-2.301,-2.295,-2.300,-2.306,-2.312,-2.295,-2.306,-2.335,-2.317,-2.309,-2.289,-2.309,-2.284,-2.297,-2.321,-2.316,-2.311,-2.330,-2.311,-2.330,-2.321,-2.303,-2.285,-2.313,-2.293,-2.302,-2.298,-2.302,-2.328,-2.310,-2.288,-2.310,-2.299,-2.320,-2.300,-2.317,-2.317,-2.307,-2.319,-2.296,-2.321,-2.293,-2.315,-2.296,-2.306,-2.320,-2.291,-2.317,-2.299,-2.300,-2.306,-2.318,-2.304,-2.321,-2.289,-2.304,-2.307,-2.318,-2.297,-2.347,-2.307,-2.309,-2.309,-2.312,-2.329,-2.306,-2.329,-2.304,-2.299,-2.264,-2.298,-2.308,-2.304,-2.307,-2.302,-2.316,-2.320,-2.326,-2.318,-2.321,-2.290,-2.333,-2.311,-2.312,-2.301,-2.305];
nivel_55mm=mean(nivel_55mm);

nivel_65mm=[-1.618,-1.593,-1.592,-1.625,-1.620,-1.608,-1.557,-1.617,-1.641,-1.640,-1.622,-1.637,-1.615,-1.619,-1.621,-1.636,-1.636,-1.642,-1.611,-1.641,-1.625,-1.631,-1.627,-1.620,-1.647,-1.619,-1.631,-1.643,-1.645,-1.637,-1.632,-1.633,-1.638,-1.636,-1.640,-1.630,-1.635,-1.642,-1.627,-1.625,-1.643,-1.645,-1.631,-1.633,-1.632,-1.634,-1.612,-1.622,-1.620,-1.636,-1.629,-1.647,-1.624,-1.633,-1.639,-1.640,-1.630,-1.614,-1.645,-1.644,-1.627,-1.647,-1.610,-1.638,-1.639,-1.635,-1.652,-1.645,-1.647,-1.638,-1.634,-1.630,-1.628,-1.627,-1.648,-1.632,-1.625,-1.625,-1.630,-1.630,-1.613,-1.621,-1.651,-1.620,-1.639,-1.633,-1.641,-1.639,-1.642,-1.631,-1.636,-1.648,-1.637,-1.625,-1.620,-1.630,-1.627,-1.617,-1.634,-1.619];
nivel_65mm=mean(nivel_65mm);

nivel_75mm=[-0.979,-0.982,-0.968,-0.961,-0.964,-0.959,-0.930,-0.954,-0.960,-0.973,-0.958,-0.956,-0.953,-0.977,-0.982,-0.978,-0.958,-0.972,-0.974,-0.938,-0.975,-0.983,-0.962,-0.964,-0.966,-0.980,-0.969,-0.942,-0.946,-0.962,-0.935,-0.952,-0.949,-0.965,-0.952,-0.946,-0.978,-0.978,-0.976,-0.958,-0.955,-0.976,-0.951,-0.950,-0.977,-0.967,-0.963,-0.958,-0.960,-0.975,-0.976,-0.953,-0.955,-0.978,-0.944,-0.972,-0.961,-0.951,-0.957,-0.956,-0.944,-0.988,-0.944,-0.955,-0.967,-0.964,-0.973,-0.976,-0.994,-0.975,-0.981,-0.961,-0.967,-0.925,-0.943,-0.941,-0.939,-0.969,-0.955,-0.989,-0.969,-0.946,-0.955,-0.965,-0.967,-0.969,-0.951,-0.948,-0.950,-0.978,-0.959,-0.959,-0.934,-0.978,-0.945,-0.920,-0.951,-0.974,-0.958,-0.981];
nivel_75mm=mean(nivel_75mm);

nivel_85mm=[-0.288,-0.279,-0.293,-0.267,-0.258,-0.273,-0.268,-0.280,-0.281,-0.308,-0.292,-0.311,-0.265,-0.253,-0.284,-0.288,-0.281,-0.285,-0.280,-0.269,-0.264,-0.274,-0.272,-0.309,-0.294,-0.292,-0.275,-0.255,-0.267,-0.308,-0.267,-0.292,-0.291,-0.287,-0.287,-0.303,-0.255,-0.290,-0.276,-0.286,-0.284,-0.270,-0.265,-0.280,-0.270,-0.268,-0.271,-0.282,-0.274,-0.288,-0.266,-0.276,-0.305,-0.261,-0.290,-0.284,-0.262,-0.297,-0.279,-0.270,-0.291,-0.283,-0.250,-0.295,-0.294,-0.304,-0.262,-0.285,-0.312,-0.292,-0.268,-0.283,-0.277,-0.239,-0.262,-0.272,-0.278,-0.284,-0.286,-0.265,-0.271,-0.261,-0.276,-0.290,-0.275,-0.247,-0.284,-0.267,-0.273,-0.284,-0.265,-0.256,-0.249,-0.273,-0.286,-0.264,-0.257,-0.240,-0.265,-0.260];
nivel_85mm=mean(nivel_85mm);

%nivel_95mm=[];
%nivel_95mm=mean(nivel_95mm);
nivel_95mm=0.4761;
nivel_105mm=[1.232,1.252,1.209,1.254,1.211,1.229,1.235,1.247,1.221,1.153,1.208,1.216,1.203,1.182,1.233,1.240,1.270,1.272,1.274,1.269,1.150,1.179,1.210,1.190,1.253,1.228,1.228,1.202,1.106,1.261,1.148,1.202,1.240,1.234,1.312,1.277,1.199,1.315,1.186,1.248,1.294,1.327,1.207,1.236,1.227,1.218,1.237,1.200,1.241,1.287,1.282,1.229,1.284,1.241,1.215,1.218,1.180,1.191,1.139,1.194,1.209,1.257,1.237,1.298,1.243,1.224,1.275,1.175,1.181,1.227,1.232,1.224,1.229,1.252,1.196,1.317,1.168,1.273,1.193,1.224,1.219,1.211,1.163,1.242,1.206,1.254,1.281,1.193,1.223,1.229,1.275,1.272,1.237,1.223,1.202,1.256,1.281,1.253,1.193,1.247];
nivel_105mm=mean(nivel_105mm);

nivel_115mm=[2.373,2.401,2.427,2.407,2.404,2.432,2.346,2.369,2.415,2.440,2.409,2.384,2.380,2.385,2.451,2.368,2.413,2.450,2.403,2.392,2.367,2.373,2.415,2.401,2.474,2.366,2.361,2.365,2.351,2.417,2.405,2.356,2.382,2.379,1.581,1.663,1.714,1.611,1.656,1.624,1.609,1.629,1.629,1.639,1.652,1.677,1.685,1.665,1.642,1.692,1.644,1.606,1.658,1.656,1.648,1.654,1.638,1.639,1.663,1.659,1.652,1.648,1.644,1.623,1.629,1.595,1.608,1.611,1.607,1.685,1.649,1.647,1.610,1.567,1.579,1.570,1.769,1.874,1.793,1.629,1.637,2.407,2.359,1.953,2.139,2.177,1.665,2.355,2.300,2.370,2.354,1.922,1.625,1.673,1.622,1.649,1.628,1.613,1.635,1.665];
nivel_115mm=mean(nivel_115mm);



c=[nivel_25mm, nivel_35mm, nivel_45mm, nivel_55mm, nivel_65mm, nivel_75mm, nivel_85mm, nivel_95mm, nivel_105mm, nivel_115mm];
c= c+31.25;




% pendiente y offset (offset=capacidad de vacio)

hold off
nivel_mm= 25:10:115;
pendiente= (c(7)-c(1))/(nivel_mm(7)-nivel_mm(1));
offset= c(7)-pendiente*nivel_mm(7);
plot(nivel_mm, nivel_mm*pendiente+offset)

% pendiente y offset (offset=capacidad de vacio)






nivel_cm= 2.5:1:11.5;

p= polyfit(nivel_cm, c, 1);     %linealizo para encontrar inversa

c_eje= 24:1:31;

nivel_medido=(c_eje-p(2))*10/p(1);      % *10 porque la linealizacion estaba en cm y ahora quiero mm. 
%nivel medido pendiente: 10/p(1), offset: -p(2)*10/p(1)

%nivelTeorico_mm= c_eje*13.46163002-338.275482;     %expresion teori%ca


%hold off
%plot(c_eje, nivelTeor
hold on
%plot(c_eje,nivel_medido)


%cryo apagado, frecuencia de muestreo 400S/s
%mm indi
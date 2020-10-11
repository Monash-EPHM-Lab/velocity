import csv
from os import listdir
from os.path import isfile, join
import math
from datetime import datetime,timedelta
import numpy as np
from scipy.ndimage import gaussian_filter as gf

import matplotlib.pyplot as plt

psd_air_raw = [1422.10903955747,474.038186365553,170.817047200565,111.911483556111,80.454454208147,56.4797733150981,48.0689504464506,42.9874746018747,34.992126635405,29.3669206365764,27.7362225254974,29.0562901414131,27.4478507116433,25.8773787298404,27.53723666634,31.2964940114622,35.1850724525551,35.3284511716584,29.7350486967415,25.2995159312504,31.7887303254462,38.7912721489205,36.7071786433957,29.7172509282961,27.0524519776827,24.5082369131053,22.179780310587,21.0034269107489,22.7812843927657,22.0878402148785,20.2334635836803,18.6501965894146,19.2404446282606,18.5886855114244,18.1685140641295,18.274781812147,19.0865291947365,19.3162545984877,18.8340939076614,18.0510633975615,18.4597303514986,18.3250222830252,19.7881290690022,18.962682735429,19.2096137477538,20.1167492683661,20.8258566331671,22.3161062785793,25.8720969494773,28.722879834472,29.0773434912,27.4574854515303,26.1081711285605,24.2756631721092,24.6666975719694,23.6600145327739,23.1934402986744,22.8674271305716,21.4387449144684,21.3906577453138,20.7531917172307,20.2938112879,19.6976055407406,19.6604954233005,19.2735608298922,18.5720383146303,18.7311095679788,18.407808141198,17.8701943295,17.293584148322,17.4828274438207,17.430569790862,16.9260151630583,16.8596595109576,17.0948252862902,17.0523632366587,16.8416584555496,16.7416000022566,16.7495444949531,16.6031989958143,16.4925611842653,16.758957482884,17.5271150491717,17.3212387126042,17.47322711489,17.7097130991187,17.1375059649857,17.1999800346944,16.9532106044511,16.686427100989,16.6404176968701,16.4111003973227,17.092597387128,16.8613247595443,16.5038575173362,16.410298762382,16.5410109729253,16.0441593696686,16.0652169176085,16.2929250843334,17.1613406396552,16.8360566815891,16.4322640341253,16.6152095701116,16.4073973442048,15.8025862293678,15.8600791847259,16.4669573971987,16.3630453147034,16.1504611243569,16.1642795256226,16.1035613106718,16.6670298201251,16.0068925746197,16.0220203859748,16.2581701622086,15.8914714549667,16.1777275158917,15.9474857280343,15.9343983874835,16.0590933756632,15.9776215937807,16.1408312757405,16.172049179312,16.2466142273884,15.8812957856928,16.3303847144999,16.845435113147]
psd_wtr_raw = [1679.11758813773,1585.23765001988,811.824453451492,484.498468827437,389.242543562682,320.241298569802,279.658395144594,277.498852497492,250.369473297577,210.79528260387,192.181569093742,179.08344863643,159.649824328974,154.493730269748,137.655592816356,121.377168231833,116.27277964966,107.899657342535,103.123548379053,93.9690859407588,81.3616829158415,72.6689924349284,68.7358538644205,60.6519103965943,57.6957506091992,53.0034400698624,47.8863973416973,49.5374916743631,43.5663493551015,38.2806305597322,38.0927308117151,34.9539395317971,33.5508568663203,31.0625464731338,30.1815965865792,29.6608587068236,28.4962616627603,28.1720958880322,28.2906248435064,28.3649926800594,27.804643624247,28.065232714604,27.6956892229523,27.2663538957787,27.1708610990139,26.4479847675681,26.1137813893795,28.0796983870495,29.7448868389517,30.6108206820489,31.1433236120318,33.0733258672698,36.0596624291327,33.852785435192,29.2441180004644,25.7383395077022,25.6227210334,24.9283760379196,22.9609563711601,22.1950834515127,22.066436631615,21.470722217515,21.2310848940224,21.2369276213988,21.2681118905493,20.9793224178652,20.6856258968838,20.6590353573103,20.1702759249163,19.6616283647439,19.5304002524956,19.2498012413606,19.965652504237,19.7070650463692,18.6374019819043,18.8922721405099,19.8896023972916,20.1273023395166,21.4176039908095,23.6995015161658,25.605057161327,26.910785033156,30.8018698218633,28.8889151996795,25.3273377919279,22.2473938943432,21.4509482320332,21.5275542486357,20.7961710984235,19.7189171743706,19.7911999922041,19.7019028738656,19.8292647441484,18.9442822320137,18.8514705216123,18.9435469905116,19.3877841638282,18.855624870566,18.404645887932,18.4045005554021,18.3353497967209,17.9444055644289,18.2527510441701,18.1806391974472,18.2906541751091,17.8402037052691,17.9992145240375,18.0224742766553,18.0489025942458,18.885038446296,18.2238308891092,18.2920144177992,17.9064035862294,17.1877034601336,17.6736528879649,17.8005238096692,17.9613986118397,17.3382378149286,17.3504803193324,17.3924012319674,18.0867431023582,17.869142065549,17.2822753177962,17.0712601023178,17.4947393389086,17.6432065554528,17.3526002015789,17.6147330118678]
psd_air_raw_8 = [3537.4181336241,825.489364428702,195.697527200199,127.539978115789,105.081327980272,89.3178622371269,82.8097276669006,80.3099595889082,79.5307894678321,76.2011800008551,70.06552444206,67.3259346129688,65.5907203090924,63.828064028528,63.947054524711,64.7993704355791,67.3051829368463,68.0807479467599,66.4848399305016,65.2176021058105,64.4676877648544,65.191791602702,65.2445857403151,61.5813753257671,59.2902310938791,55.0888788104066,54.0471022100648,52.5136517918245,52.2302011080965,49.8257516682081,49.6187575574219,49.5157802613102,51.3380619840603,52.6438771175792,59.2713480720046,65.3535245046399,74.862243925656,83.3121276991126,82.4802419468271,72.9107939929461,65.189947076506,59.9680079292284,55.5135545209632,53.4660632107858,53.2782835463579,50.494842530293,48.4088929885137,47.4517782406313,47.5550690886127,45.5899559019266,45.6248561205702,45.2193563168947,44.5326162581478,43.8255469911634,42.9054464574637,41.9904820935031,42.288687253719,41.7041086942344,40.794930135453,40.491438046995,40.0843094302496,39.7735152357011,40.0873622187728,40.3267854462988,41.661598727398,41.5245144902694,39.620955104091,38.97274682661,39.0315441009268,39.0821552170852,38.873783363014,39.2333114244092,40.0021632958038,39.2136753898582,38.1638704062248,38.4651694940312,39.3026989052122,38.7971499499256,38.3773034293252,37.9490311023502,38.2609175691138,39.3746681701128,41.3776953353644,37.8536033814621,37.9333278306615,37.0474885216662,36.894393190072,37.7180910294926,37.7841167875119,37.4855646545111,36.9014865068108,37.3738520422671,38.1672790651072,37.6154425633544,37.1891895119864,36.9857495240686,37.8046509509442,36.6359550589911,36.5988121469561,37.0045122237658,37.6424424707316,36.8396750180876,37.0606542618743,36.7440145724355,37.5786184359563,36.1954797159381,36.6719066897228,36.3755503048925,37.2750446128889,38.1399932004605,37.2602270095332,37.0740739117289,36.9286295829317,36.1109785769158,36.2148111035111,36.8365721076603,37.8164716825814,37.0983964914962,36.1011948405082,36.3589772639244,37.7496841121642,36.4426626658279,36.9378629621563,36.4671299501613,37.4259433640398,37.2990662910623,37.2136632530451,37.4971805801588]
psd_wtr_raw_8 = [41174.3806571184,120302.602855875,105542.51888193,90790.7283814576,81138.5071046505,73497.634034415,70871.6169214304,67832.8440412094,64030.6565793227,62377.2545196615,57387.5758371296,54877.5123593713,51662.8890798747,49924.2000253269,48066.6035051467,46206.5451444119,45825.6249444549,45519.0761679827,42877.4449842415,39038.8759014401,39801.8840654472,39522.2782636183,38690.9386405924,32499.6977513894,31517.7858227661,30574.3168116653,31491.6907460801,28199.7634981957,26771.6796665609,25658.0470493405,25255.7432518548,25295.4667431756,22607.5989125187,21664.9992686881,21882.2862486979,18317.1679943823,17967.2287901649,17004.5694331945,16689.7458757698,16162.9415214433,15383.1042421148,14129.1571520087,13365.8725878357,13144.7707812285,12175.1231498155,11163.0223285944,10364.0835204271,9277.39846132959,8880.8058553402,8293.85291245132,7738.12839138987,7442.59433910662,6498.25916247648,6173.42788624381,6168.56250250599,5539.4987722919,4942.32996900016,4741.84010877772,4362.67154743644,4002.48676172361,3689.12969042127,3088.60311562455,2728.0585858244,2645.94903551163,2536.34154505779,2276.8166942729,1989.99342275647,1834.9045965329,1613.01994257326,1544.05316037096,1394.45333450635,1219.36024234386,1093.67091818114,967.885479907105,858.916807080735,786.472013380069,703.851534742971,611.150499342732,545.295920224207,498.858467275982,444.697343350288,394.33628009155,329.785225174501,303.20266167737,277.697751284757,239.873534126809,214.906837063002,191.611160362745,177.413227081408,161.550563500485,150.251996239125,141.331001512586,141.351924957814,136.102055464381,132.19053554118,122.670316174744,132.343368171962,131.243233870195,139.08421425084,143.622684000841,154.622202104278,169.529710370059,177.607931915826,185.50510537213,221.058025769715,262.883489133994,297.095312212417,328.856497902624,309.75132687907,267.451671272364,270.468511455626,295.394565391699,318.864275830957,316.287007249298,329.424698080006,335.327677403852,346.09796609361,389.543511256564,399.462940904629,408.585659096312,392.201792346997,406.39977506655,422.653415793692,420.817597939117,401.587214344958,418.19420242922,422.190945973916,425.602365794274]


psd_air = [x/(sum(psd_air_raw)/len(psd_air_raw)) for x in psd_air_raw]
psd_wtr = [x/(sum(psd_wtr_raw)/len(psd_wtr_raw)) for x in psd_wtr_raw]
psd_wtr_8 = [x/(sum(psd_wtr_raw_8)/len(psd_wtr_raw_8)) for x in psd_wtr_raw_8]
psd_air_8 = [x/(sum(psd_air_raw_8)/len(psd_air_raw_8)) for x in psd_air_raw_8]




# for i in range (8):
    # for j in range(16):
        # print("{:.4f}".format(psd_wtr[16*i+j]**-1),end = ', ')
    # print()



class SdPoint:
        binconv = 69.7 *0.75  /( 256 / 128) *1.15
        coslos = 1.15
        
        def __init__(self):
                self.fft = [None]*128

                self.idv = None
                self.vem = 0
                self.ves = 0
                self.vea = 0
                self.time = datetime(1, 1, 1)
                self.scale = 0
                self.hach_vel = 0
                self.hach_depth = 0
                self.thrsh = 8000
                self.ctlow = -2000
                self.cthigh = -800


        def match (self, idv, vem, ves):
                if (idv == self.idv) and (vem == self.vem) and (ves == self.ves):
                        return True
                else:
                        return False

        def get_fft (self):
                return self.fft
        def set_fft (self, fft, i):
                self.fft[i] = fft
        def get_id (self):
                return self.idv
        def set_id (self, idv):
                self.idv = idv
        def get_vem (self):
                return self.vem
        def get_vemc (self):
                return self.vem * SdPoint.coslos
        def set_vem (self, vem):
                self.vem = vem
        def get_ves (self):
                return self.ves
        def get_vesc (self):
                return self.ves * SdPoint.coslos
        def set_ves (self, ves):
                self.ves = ves
        def get_vea (self):
                return self.vea
        def set_vea (self, vea):
                self.vea = vea
        def get_time (self):
                return self.time
        def set_time (self, time):
                self.time = time
        def get_scale (self):
                return self.scale
        def set_scale (self, scale):
                self.scale = scale
        def get_hach_vel (self):
                return self.hach_vel
        def set_hach_vel (self, vel):
                self.hach_vel = vel
        def get_hach_depth(self):
                return self.hach_depth
        def set_hach_depth(self, depth):
                self.hach_depth = depth
        
        def collect(self, plist):
            
            try:
                self.fft = [np.sqrt(np.mean(np.array([x.get_fft()[i] for x in plist])**2)) for i in range(128)]
            
                #self.idv = np.mean([x.idv for x in plist])
                self.vem = np.mean([x.vem for x in plist])
                self.ves = np.mean([x.ves for x in plist])
                self.time = plist[0].time
                self.scale = np.mean([x.scale for x in plist])
                #self.hach_vel = np.mean([x.hach_vel for x in plist])
                self.hach_vel = plist[0].hach_vel
                self.thrsh = np.mean([x.thrsh for x in plist])
            
            except TypeError:
                pass

        def cannym(self):

            if self.fft[0] == None:
               return 0, 0, 0 , 0
            
            
            max_bin = 0
            #whiten noise
            fftpsd = [binn/weight for binn,weight in zip(self.fft, psd_air_8)]
            
            #gausian
            fftpsd = gf(fftpsd, sigma = 1)
            
            #sobel derivative
            sob = [-1, 0 ,1]
            
            # dfft = [None] * 128
            dfft = []
            
            for i in range(len(fftpsd)):
                try:
                    val =  sum([x*y for x,y in zip(sob,fftpsd[i-1:i+2])])
                except IndexError:
                    val = 0
                dfft.append(val)
                
            for i,val in reversed(list(enumerate(dfft))):
                if i > 64:
                    pass
                elif val > self.ctlow:
                    pass
                elif val < dfft[i-1]:
                     max_bin = i
                     break
                else:
                    pass
           
            max_val = dfft[max_bin]
            
            
            
            mean_indx = 0
            std_indx = 0
     
            #remove negative
            dfft = [(0 if (x > 0.2*max_val) else x) for x in dfft]

            #isolate peak right and left
            is_peak = True
            for i,x in enumerate(dfft):
                if i < max_bin:
                        pass
                else:
                        if (is_peak and dfft[i] < 0):
                                pass
                        else:
                                is_peak = False
                                dfft[i] = 0
                                

            
            is_peak = True
            for i,x in reversed(list(enumerate(dfft))):
                if i > max_bin:
                        pass
                else:
                        if (is_peak and dfft[i] < 0):
                                pass
                        else:
                                is_peak = False
                                dfft[i] = 0


            #find mean and stdev
            try:
                mean_indx = np.average(list(range(2,64)), weights = dfft[2:64])
                std_indx = math.sqrt(np.average((list(range(2,64))-mean_indx)**2, weights=dfft[2:64]))
            except ZeroDivisionError:
                mean_indx = 0
                std_indx = 0
           
            
            
            mean_indx *= SdPoint.binconv
            std_indx *= SdPoint.binconv
            return mean_indx, std_indx, max_val ,dfft

        def canny(self):
            if self.fft[0] == None:
               return 0, 0
            
            
            mean_indx = 0
            #whiten noise
            fftpsd = [binn/weight for binn,weight in zip(self.fft, psd_wtr)]
            
            #gausian
            fftpsd = gf(fftpsd, sigma = 2.5)
            
            #sobel derivative
            sob = [-1, 0 ,1]
            
            # dfft = [None] * 128
            dfft = []
            
            for i in range(len(fftpsd)):
                try:
                    val =  sum([x*y for x,y in zip(sob,fftpsd[i-1:i+2])])
                except IndexError:
                    val = 0
                dfft.append(val)
                
            for i,val in reversed(list(enumerate(dfft))):
                if i > 64:
                    pass
                elif val > self.ctlow:
                    mean_indx = i
                    pass
                elif val < dfft[i-1]:
                     mean_indx = i
                     break
                else:
                    pass
           
            # mean_indx = dfft.index(min(dfft[2:64]))
            std_indx = 0
     
            
            
            mean_indx *= SdPoint.binconv
            std_indx *= SdPoint.binconv
            return mean_indx, std_indx, dfft

        def romeM(self):
        
            if self.fft[0] == None:
               return 0, 0, 0
               
            fftpsd = [x for x in self.fft]
            # fftpsd[0] = 0
            # fftpsd[1] = 0
            fftpsd = [binn/weight for binn,weight in zip(fftpsd, psd_wtr)]
            fftpsd = np.convolve(fftpsd, np.ones((4,))/4, mode='same')
            fftpsd = [x for x in fftpsd]

            
            return self.romeATM(fftpsd)
            
        def romeATM(self, fft = None):
            indx_low = 3
            indx_high = 64
            threshold = self.thrsh
            
            low_bin = indx_low
            high_bin = indx_high
        
            if fft is None:
                fft = self.fft

            if fft[0] == None:
               return 0, 0
            #find max bin
            
            ampmax = max(fft[indx_low:indx_high])
            
            max_bin = fft.index(ampmax)

            threshold = 0.8*ampmax

            #remove negative
            fft = [(0 if (x < threshold) else x) for x in fft]
            
            

            #isolate peak right and left
            is_peak = True
            for i,x in enumerate(fft):
                if i < max_bin:
                        pass
                else:
                        if (is_peak and fft[i] > 0):
                                high_bin = i
                        else:
                                is_peak = False
                                fft[i] = 0
                                

            
            is_peak = True
            for i,x in reversed(list(enumerate(fft))):
                if i > max_bin:
                        pass
                else:
                        if (is_peak and fft[i] > 0):
                                low_bin = i
                        else:
                                is_peak = False
                                fft[i] = 0

                vea = ampmax#*(low_bin - high_bin)
                mean_indx = (low_bin + high_bin)/2
                std_indx = low_bin - high_bin
            
            
            mean_indx *= SdPoint.binconv
            std_indx *= SdPoint.binconv
                       
            return mean_indx, std_indx, vea
            
            
        def algoM(self):
        
            if self.fft[0] == None:
               return 0, 0, 0
               
            fftpsd = [x for x in self.fft]
            # fftpsd[0] = 0
            # fftpsd[1] = 0
            fftpsd = [binn/weight for binn,weight in zip(fftpsd, psd_wtr)]
            #fftpsd = np.convolve(fftpsd, np.ones((8,))/8, mode='same')
            fftpsd = [x for x in fftpsd]

            
            return self.algoATM(fftpsd)
        
        def algoATM(self, fft = None):
            indx_low = 3
            indx_high = 64
            threshold = self.thrsh
        
            if fft is None:
                fft = self.fft

            if fft[0] == None:
               return 0, 0
            #find max bin
            
            ampmax = max(fft[indx_low:indx_high])
            
            max_bin = fft.index(ampmax)

            
            if (max_bin < 8):
                upmax = max(fft[8:16])
                if (upmax > threshold):
                    ampmax = upmax
                    max_bin = fft.index(ampmax)

            #set theshold
            ##fft = [x - self.thrsh for x in fft]

            #remove negative
            fft = [(0 if (x < threshold) else x) for x in fft]

            #isolate peak right and left
            is_peak = True
            for i,x in enumerate(fft):
                if i < max_bin:
                        pass
                else:
                        if (is_peak and fft[i] > 0):
                                pass
                        else:
                                is_peak = False
                                fft[i] = 0
                                

            
            is_peak = True
            for i,x in reversed(list(enumerate(fft))):
                if i > max_bin:
                        pass
                else:
                        if (is_peak and fft[i] > 0):
                                pass
                        else:
                                is_peak = False
                                fft[i] = 0


            #find mean and stdev
            try:
                vea = ampmax #sum(fft[indx_low:indx_high])
                mean_indx = np.average(list(range(indx_low,indx_high)), weights = fft[indx_low:indx_high])
                std_indx = math.sqrt(np.average((list(range(indx_low,indx_high))-mean_indx)**2, weights=fft[indx_low:indx_high]))
            except ZeroDivisionError:
                mean_indx = 0
                std_indx = 0
            mean_indx *= SdPoint.binconv
            std_indx *= SdPoint.binconv
                       
            return mean_indx, std_indx, vea

        def algo(self):

            if self.fft[0] == None:
               return 0, 0, 0
               
            fftpsd = [x for x in self.fft]
            # fftpsd[0] = 0
            # fftpsd[1] = 0
            fftpsd = [binn/weight for binn,weight in zip(fftpsd, psd_air_8)]
            # fftpsd = gf(fftpsd, sigma = .3)
            fftpsd = [x for x in fftpsd]
            
            
            
            return self.algoAT(fftpsd)
        
        def algoAT(self, fft = None):
            if fft is None:
                fft = self.fft

            if fft[0] == None:
               return 0, 0
            #find max bin
            
            ampmax = max(fft[2:64])
            
            max_bin = fft.index(ampmax)


            #set theshold
            ##fft = [x - self.thrsh for x in fft]

            #remove negative
            fft = [(0 if (x < self.thrsh) else x) for x in fft]

            #isolate peak right and left
            is_peak = True
            for i,x in enumerate(fft):
                if i < max_bin:
                        pass
                else:
                        if (is_peak and fft[i] > 0):
                                pass
                        else:
                                is_peak = False
                                fft[i] = 0
                                

            
            is_peak = True
            for i,x in reversed(list(enumerate(fft))):
                if i > max_bin:
                        pass
                else:
                        if (is_peak and fft[i] > 0):
                                pass
                        else:
                                is_peak = False
                                fft[i] = 0


            #find mean and stdev
            try:
                vea = sum(fft[2:64])
                mean_indx = np.average(list(range(2,64)), weights = fft[2:64])
                std_indx = math.sqrt(np.average((list(range(2,64))-mean_indx)**2, weights=fft[2:64]))
            except ZeroDivisionError:
                mean_indx = 0
                std_indx = 0
            mean_indx *= SdPoint.binconv
            std_indx *= SdPoint.binconv
            return mean_indx, std_indx, vea

def load_data(points):

    device = '000'
    sp_st = 3

    hach_date = []
    hach_depth = []
    hach_vel = []
    
    
    hach_files = ['data/hach/' + f for f in listdir('data/hach') if isfile(join('data/hach', f))]
    log_files = ['data/log_' + device + '/' + f for f in listdir('data/log_' + device) if isfile(join('data/log_' + device, f))]
    sd_files = ['data/sd_'  + device + '/' + f for f in listdir('data/sd_' + device) if isfile(join('data/sd_' + device, f))]\
    
    hach_files = hach_files[sp_st:9]#9
    log_files = log_files[sp_st:]
    sd_files = sd_files[sp_st:]
    
    print([x for x in zip(hach_files, log_files, sd_files)])
    
    for hach_f,log_f,sd_f in zip(hach_files, log_files, sd_files):
        point_bach = []
    
        with open(hach_f, newline='') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
                for i,row in enumerate(spamreader):
                        if i < 5:
                                continue
                        date_string = row[0] + " " + row[2]
                        hach_date.append(datetime.strptime(date_string, '%d/%b/%y %H:%M'))
                        hach_depth.append(float(row[12]))
                        hach_vel.append(float(row[17])*1000)

        with open(log_f, newline='') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for i,row in enumerate(spamreader):

                        try:
                            point_bach.append(SdPoint())
                            
                            point_bach[i].set_time(datetime.strptime(row[0], '%d/%m/%y %I:%M:%S %p')+timedelta(hours=10))
                            point_bach[i].set_id(float(row[1]))
                            point_bach[i].set_vem(float(row[7])) 
                            point_bach[i].set_ves(float(row[8]))
                            point_bach[i].set_vea(float(row[9]))                            
                        except ValueError:
                            pass
        hi = 0;
        skip = False

        for point in point_bach:
                
                try:
                        boslT = point.get_time()
                        hachT  = hach_date[hi]
                        Dtime = boslT - hachT


                        while (Dtime >= timedelta(minutes=1)):
                                hi += 1
                                boslT = point.get_time()
                                hachT  = hach_date[hi]
                                Dtime = boslT - hachT
                                


                        while (Dtime <= timedelta(minutes=-1)):
                                hi -= 1
                                boslT = point.get_time()
                                hachT  = hach_date[hi]
                                Dtime = boslT - hachT
                                

                                if hi < 0:
                                        hi = 0
                                        skip = True
                                        break;
                                

                
                        point.set_hach_vel(hach_vel[hi])
                        point.set_hach_depth(hach_depth[hi])
                except IndexError:
                                break

                if skip:
                        skip = False
                        continue

        hi = 0;
        skip = False
        with open(sd_f, newline='') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for i,row in enumerate(spamreader):
                    try:
                            idsd = float(row[0])
                            vemsd = float(row[136])
                            vessd = float(row[137])
                    except IndexError:
                            continue
                    
                    try:
                        point = point_bach[hi]
                    
                    
                        while(point.get_id() < idsd):
                                hi += 1;
                                point = point_bach[hi]

                        while(point.get_id() > idsd):
                                hi -= 1;
                                point = point_bach[hi]

                                if hi < 0:
                                        hi = 0
                                        skip = True
                                        break;  
                    except IndexError:
                        pass

                    if skip:
                            skip = False
                            continue

                    if (point.match(idsd,vemsd,vessd)):
                            for j in range(128):
                                    point.set_fft(float(row[j+3]),j)
                            point.set_scale(float(row[132]))
                                
        points += point_bach                        
         
    return points



def load_test(points):
    with open('FFT.csv', newline='') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
                for i,row in enumerate(spamreader):
                            
                        try:
                                idsd = float(row[0])
                                vemsd = float(row[136])
                                vessd = float(row[137])
                                veasd = float(row[138])
                        except IndexError:
                                continue
                        
                        point = SdPoint()
                        
                        point.set_vem(vemsd)
                        point.set_ves(vessd)
                        point.set_vea(veasd)

                        for j in range(128):
                                point.set_fft(float(row[j+3]),j)
                        point.set_scale(float(row[132]))
                        
                        points.append(point)
    return points
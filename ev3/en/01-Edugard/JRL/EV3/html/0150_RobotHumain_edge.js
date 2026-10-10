/*jslint */
/*global AdobeEdge: false, window: false, document: false, console:false, alert: false */
(function (compId) {

    "use strict";
    var im='images/',
        aud='media/',
        vid='media/',
        js='js/',
        fonts = {
            'Digital-7': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            '\'Digital tech\'': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            'MagistralC': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />'        },
        opts = {
            'gAudioPreloadPreference': 'auto',
            'gVideoPreloadPreference': 'auto'
        },
        resources = [
        ],
        scripts = [
            js+"jquery-2.0.3.min.js",
            js+"jquery.ui.touch-punch.min.js",
            js+"jquery-ui.min.js"
        ],
        symbols = {
            "stage": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "height",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['80px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'VS',
                            type: 'group',
                            rect: ['304px', '99px', '955', '805', 'auto', 'auto'],
                            c: [
                            {
                                id: 'A1_versus',
                                type: 'image',
                                rect: ['60px', '17px', '895px', '788px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_versus.png",'0px','0px']
                            },
                            {
                                id: 'Text7',
                                type: 'text',
                                rect: ['578px', '162px', '329px', '113px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​</p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'muscle',
                                type: 'ellipse',
                                rect: ['335px', '287px', '33px', '37px', 'auto', 'auto'],
                                borderRadius: ["50%", "50%", "50%", "50%"],
                                fill: ["rgba(255,0,0,1.00)"],
                                stroke: [0,"rgba(0,0,0,1)","none"]
                            },
                            {
                                id: 'Gr_msucles',
                                type: 'group',
                                rect: ['25px', '117px', '307', '174', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'muscles',
                                    type: 'image',
                                    rect: ['-99px', '-20px', '399px', '195px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"arrow_l_up4.png",'0px','0px']
                                },
                                {
                                    id: 'Text2',
                                    type: 'text',
                                    rect: ['-66px', '12px', '338px', '94px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 30px; font-weight: 700;\">An electric motor </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 30px;\">plays the role of the muscles.</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'a1_moteur',
                                    type: 'image',
                                    rect: ['-59px', '316px', '251px', '251px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"a1_moteur.jpg",'0px','0px']
                                },
                                {
                                    id: 'a1_muscle',
                                    type: 'image',
                                    rect: ['769px', '283px', '252px', '272px', 'auto', 'auto'],
                                    clip: 'rect(0px 252px 248px 0px)',
                                    fill: ["rgba(0,0,0,0)",im+"a1_muscle.jpg",'0px','0px']
                                }]
                            },
                            {
                                id: 'Gr_coeur',
                                type: 'group',
                                rect: ['13px', '183px', '1062px', '518px', 'auto', 'auto'],
                                clip: 'rect(0px 1057.7099609375px 517.3623046875px -34px)',
                                c: [
                                {
                                    id: 'heart',
                                    type: 'image',
                                    rect: ['-6px', '12px', '458px', '244px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CASE_BULLES_DROITE_DESSUS.png",'0px','0px']
                                },
                                {
                                    id: 'Text',
                                    type: 'text',
                                    rect: ['33px', '92px', '377px', '113px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 28px; font-weight: 700;\">A battery </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 28px;\">distributes the energy for all the elements of the robot (It plays the role of a human heart)</span></p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'a1_heart',
                                    type: 'image',
                                    rect: ['739px', '231px', '297px', '297px', 'auto', 'auto'],
                                    clip: 'rect(32px 253px 277px 56px)',
                                    fill: ["rgba(0,0,0,0)",im+"a1_heart.jpg",'0px','0px'],
                                    transform: [[],[],[],['1','0.91107']]
                                },
                                {
                                    id: 'a1_battery',
                                    type: 'image',
                                    rect: ['-39px', '285px', '235px', '235px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"a1_battery.jpg",'0px','0px']
                                }]
                            },
                            {
                                id: 'Gr_cerveau_',
                                type: 'group',
                                rect: ['514px', '43px', '360', '189', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'brains',
                                    type: 'image',
                                    rect: ['0px', '0px', '392px', '246px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"arrow_r_dn2.png",'0px','0px']
                                },
                                {
                                    id: 'Text3',
                                    type: 'text',
                                    rect: ['42px', '78px', '342px', '133px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 30px; font-weight: 700;\">A programmable electronic</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 30px;\"> card plays the role of a human brain.</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'a1_brain',
                                    type: 'image',
                                    rect: ['95px', '304px', '612px', '470px', 'auto', 'auto'],
                                    clip: 'rect(94px 448px 349.3956604003906px 168px)',
                                    fill: ["rgba(0,0,0,0)",im+"a1_brain.jpg",'0px','0px'],
                                    transform: [[],[],[],['0.89554','0.89554']]
                                },
                                {
                                    id: 'a1_carte',
                                    type: 'image',
                                    rect: ['-644px', '337px', '500px', '360px', 'auto', 'auto'],
                                    clip: 'rect(12px 470px 340px 28px)',
                                    fill: ["rgba(0,0,0,0)",im+"a1_carte.jpg",'0px','0px'],
                                    transform: [[],[],[],['0.61585','0.61585']]
                                }]
                            },
                            {
                                id: 'Gr_yeux_',
                                type: 'group',
                                rect: ['509px', '82px', '370', '126', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'eye',
                                    type: 'image',
                                    rect: ['0px', '0px', '370px', '195px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"arrow_r_dn2.png",'0px','0px']
                                },
                                {
                                    id: 'Text4',
                                    type: 'text',
                                    rect: ['35px', '60px', '307px', '56px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 31px; font-weight: 700;\">Cameras</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 31px;\"> plays the role of the eyes of a huamin cops.</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'a1_eye',
                                    type: 'image',
                                    rect: ['-12px', '184px', '792px', '612px', 'auto', 'auto'],
                                    clip: 'rect(172px 636px 430px 136px)',
                                    fill: ["rgba(0,0,0,0)",im+"a1_eye.jpg",'0px','0px'],
                                    transform: [[],[],[],['0.59','0.57364']]
                                },
                                {
                                    id: 'a1_camera',
                                    type: 'image',
                                    rect: ['-570px', '378px', '285px', '223px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"a1_camera.jpg",'0px','0px']
                                }]
                            },
                            {
                                id: 'Gr_bouche',
                                type: 'group',
                                rect: ['508px', '123px', '411', '117px', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'mouths',
                                    type: 'image',
                                    rect: ['0px', '0px', '421px', '236px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"arrow_r_dn2.png",'0px','0px']
                                },
                                {
                                    id: 'Text8',
                                    type: 'text',
                                    rect: ['43px', '72px', '362px', '105px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 28px; font-weight: 700;\">A speaker </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 28px;\">emits the sounds so that the robot can communicate with the outside world.</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'a1_speaking',
                                    type: 'image',
                                    rect: ['225px', '309px', '322px', '279px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"a1_speaking.png",'0px','0px']
                                },
                                {
                                    id: 'a1_bafme',
                                    type: 'image',
                                    rect: ['-529px', '323px', '235px', '279px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"a1_bafme.jpg",'0px','0px']
                                }]
                            },
                            {
                                id: 'Gr_oreilles',
                                type: 'group',
                                rect: ['553px', '91px', '390', '145', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'ears',
                                    type: 'image',
                                    rect: ['0px', '0px', '399px', '192px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"arrow_r_dn2.png",'0px','0px']
                                },
                                {
                                    id: 'Text6',
                                    type: 'text',
                                    rect: ['42px', '67px', '334px', '68px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 29px; font-weight: 700;\">A microphone </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 29px;\">plays the role of the ears of a human body</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'a1_ear',
                                    type: 'image',
                                    rect: ['123px', '241px', '450px', '442px', 'auto', 'auto'],
                                    clip: 'rect(90px 324px 352px 124px)',
                                    fill: ["rgba(0,0,0,0)",im+"a1_ear.jpg",'0px','0px']
                                },
                                {
                                    id: 'a1_micro',
                                    type: 'image',
                                    rect: ['-625px', '295px', '344px', '344px', 'auto', 'auto'],
                                    clip: 'rect(0px 250px 344px 104px)',
                                    fill: ["rgba(0,0,0,0)",im+"a1_micro.jpg",'0px','0px'],
                                    transform: [[],['16'],[],['0.82872','0.73092']]
                                }]
                            },
                            {
                                id: 'Ellipse2',
                                type: 'ellipse',
                                rect: ['474px', '174px', '39px', '37px', 'auto', 'auto'],
                                borderRadius: ["50%", "50%", "50%", "50%"],
                                fill: ["rgba(255,0,0,1)"],
                                stroke: [0,"rgba(63,25,25,1.00)","solid"]
                            },
                            {
                                id: 'mouth',
                                type: 'ellipse',
                                rect: ['483px', '98px', '33px', '25px', 'auto', 'auto'],
                                borderRadius: ["50%", "50%", "50%", "50%"],
                                fill: ["rgba(255,0,0,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'ear',
                                type: 'ellipse',
                                rect: ['530px', '70px', '23px', '25px', 'auto', 'auto'],
                                borderRadius: ["50%", "50%", "50%", "50%"],
                                fill: ["rgba(255,0,0,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'brain',
                                type: 'ellipse',
                                rect: ['483px', '18px', '33px', '25px', 'auto', 'auto'],
                                borderRadius: ["50%", "50%", "50%", "50%"],
                                fill: ["rgba(255,0,0,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'eyes',
                                type: 'ellipse',
                                rect: ['485px', '56px', '31px', '25px', 'auto', 'auto'],
                                borderRadius: ["50%", "50%", "50%", "50%"],
                                fill: ["rgba(255,0,0,1)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['478px', '37px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'TextCopy',
                            type: 'group',
                            rect: ['485px', '160px', '808px', '181px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ROBOT08Copy',
                                type: 'image',
                                rect: ['-224px', '-30px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_GREENCopy',
                                type: 'image',
                                rect: ['-56px', '-45px', '837px', '170px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'TextIN',
                                type: 'text',
                                rect: ['-5px', '-27px', '775px', '141px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">A robot automatically performs tasks that mimic human actions. Let's find out about the similarities between a robot and the human body</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'robotvshumain',
                            type: 'image',
                            rect: ['332px', '162px', '961px', '713px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"robotvshumain.png",'0px','0px']
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['-202px', '808px', '1040px', '82px', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            transform: [[],[],[],['1.14366']],
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy',
                                type: 'text',
                                rect: ['921px', '17px', '142px', '44px', 'auto', 'auto'],
                                clip: 'rect(0px 140.86962890625px 44px 0px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​discover<span style=\"font-size: 23px; font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif; font-weight: 900;\"></span></p><p style=\"margin: 0px;\"><span style=\"font-size: 23px; font-family: 'Lucida Sans Unicode', 'Lucida Grande', sans-serif; font-weight: 900;\"></span></p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        }
                    ],
                    style: {
                        '${Stage}': {
                            isStage: true,
                            rect: ['null', 'null', '1650px', '1080', 'auto', 'auto'],
                            overflow: 'hidden',
                            fill: ["rgba(240,240,240,0)"]
                        }
                    }
                },
                timeline: {
                    duration: 5500,
                    autoPlay: false,
                    data: [
                        [
                            "eid342",
                            "height",
                            5500,
                            0,
                            "linear",
                            "${valider}",
                            '82px',
                            '82px'
                        ],
                        [
                            "eid341",
                            "width",
                            5500,
                            0,
                            "linear",
                            "${valider}",
                            '1040px',
                            '1040px'
                        ]
                    ]
                }
            },
            "TITRE_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['27px', '-11px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(0, 61, 65); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">robot vs human body</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['27px', '-11px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">robot vs human body</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            id: 'Text6'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5250,
                    autoPlay: true,
                    data: [
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,57,0],
                            [0,60,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,57,0],
                            [0,100,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,100,57,0],
                            [0,140,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,57,0],
                            [0,160,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,57,0],
                            [0,190,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,57,0],
                            [0,220,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,57,0],
                            [0,260,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,57,0],
                            [0,270,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,57,0],
                            [0,290,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,57,0],
                            [0,320,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,57,0],
                            [0,350,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,57,0],
                            [0,380,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,57,0],
                            [0,405,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,57,0],
                            [0,475,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,57,0],
                            [0,505,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,505,57,0],
                            [0,530,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid199",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,0],
                            [0,540,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid201",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,540,57,0],
                            [0,565,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid203",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,565,57,0],
                            [0,595,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid205",
                            "clip",
                            5000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,0],
                            [0,680.7080078125,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0150_RobotHumain_edgeActions.js");
})("EDGE-6689876");

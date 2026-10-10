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
                scaleToFit: "both",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['85px', '0px', '1440px', '1080px', 'auto', 'auto'],
                            clip: 'rect(0px 1440px 1080px 0px)',
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px'],
                            transform: [[],[],[],['1.00694']]
                        },
                        {
                            id: 'Reflexion',
                            type: 'group',
                            rect: ['736px', '213px', '843', '745', 'auto', 'auto'],
                            clip: 'rect(0px 758.625px 745px 0px)',
                            c: [
                            {
                                id: 'reflexion',
                                type: 'image',
                                rect: ['149px', '257px', '651px', '475px', 'auto', 'auto'],
                                clip: 'rect(18.181884765625px 651px 475px 36.36376953125px)',
                                fill: ["rgba(0,0,0,0)",im+"reflexion.png",'0px','0px'],
                                transform: [[],[],[],['0.86866','0.86866']]
                            },
                            {
                                id: 'Compte_rebours',
                                type: 'group',
                                rect: ['109px', '-86px', '311px', '296px', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'timer',
                                    type: 'image',
                                    rect: ['87px', '338px', '268px', '249px', 'auto', 'auto'],
                                    clip: 'rect(-10.733642578125px 268px 262.48748779296875px 1.01953125px)',
                                    fill: ["rgba(0,0,0,0)",im+"timerTransp.png",'0px','0px'],
                                    transform: [[],[],[],['0.5603','0.35816']]
                                },
                                {
                                    id: 'secondes',
                                    type: 'text',
                                    rect: ['203px', '441px', '111px', '90px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">00​</p>",
                                    align: "center",
                                    font: ['Digital-7', [52, "px"], "rgba(41,201,121,1.00)", "900", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'minutes',
                                    type: 'text',
                                    rect: ['165px', '441px', '111px', '90px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(90, 191, 107);\">&nbsp; </span><span style=\"font-size: 50px;\">0​</span>:</p>",
                                    align: "left",
                                    font: ['Digital-7', [52, "px"], "rgba(41,201,121,1.00)", "900", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'points',
                                    type: 'text',
                                    rect: ['174px', '441px', '111px', '90px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(90, 191, 107);\">&nbsp; &nbsp;</span><span style=\"font-size: 50px;\">​</span>:</p>",
                                    align: "left",
                                    font: ['Digital-7', [52, "px"], "rgba(41,201,121,1.00)", "900", "none", "", "break-word", "normal"]
                                }]
                            }]
                        },
                        {
                            id: 'solu',
                            type: 'group',
                            rect: ['-157px', '358px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            transform: [[],[],[],['1','1.09756']],
                            c: [
                            {
                                id: 'ORANGE2',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREY2',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text32',
                                type: 'text',
                                rect: ['910px', '18px', '123px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Solution</p>",
                                align: "left",
                                font: ['MagistralC', [28, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'demonstration',
                            type: 'group',
                            rect: ['927px', '410px', '677', '381', 'auto', 'auto'],
                            c: [
                            {
                                id: 'video',
                                symbolName: 'video',
                                type: 'rect',
                                rect: ['-855px', '-415px', '1920', '1080', 'auto', 'auto'],
                                clip: 'rect(97.5px 1920px 962.66650390625px 0px)'
                            },
                            {
                                id: 'cache_cadre',
                                type: 'image',
                                rect: ['-847px', '468px', '1450px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                            },
                            {
                                id: 'full_screen',
                                type: 'image',
                                rect: ['-118px', '446px', '50px', '50px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Ex1_son',
                            type: 'video',
                            tag: 'video',
                            rect: ['180px', '253px', '1280', '720', 'auto', 'auto'],
                            clip: 'rect(45.33349609375px 1020px 672.3174438476563px 262.6669921875px)',
                            source: [vid+"mm_solutionselector.webm"],
                            preload: 'auto',
                            transform: [[],[],[],['0.97878','0.97878']]
                        },
                        {
                            id: 'Intro',
                            type: 'group',
                            rect: ['251px', '110px', '1014px', '177', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['152px', '22px', '862px', '166px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_BLUE1.png",'0px','0px']
                            },
                            {
                                id: 'ROBOT07',
                                type: 'image',
                                rect: ['15px', '34px', '160px', '163px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotP.png",'0px','0px']
                            },
                            {
                                id: 'Text2',
                                type: 'text',
                                rect: ['205px', '60px', '795px', '86px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: capitalize; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">é<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; font-size: 34px;\">crire un programme qui permet de prononcer</span></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"> la couleur detecté.&nbsp;</p>",
                                align: "left",
                                font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Hint',
                            type: 'group',
                            rect: ['911px', '247px', '656px', '266', 'auto', 'auto'],
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'Hint_green',
                                type: 'image',
                                rect: ['-185px', '-40px', '656px', '0px', 'auto', 'auto'],
                                clip: 'rect(0px 656px 241.58349609375px 0px)',
                                opacity: '0.3',
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px'],
                                transform: [[],[],[],['1','0.53765']]
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['-111px', '40px', '550px', '123px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Penser à utiliser le bloc selecteur</p>",
                                align: "left",
                                font: ['\'Arial Black\', Gadget, sans-serif', [24, "px"], "rgba(0,0,0,0.00)", "900", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'indices',
                                type: 'image',
                                rect: ['58px', '-32px', '150px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"indices.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'demo',
                            type: 'group',
                            rect: ['-154px', '360px', '1051px', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1054.28564453125px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['0.98953','1.10234']],
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
                                id: 'Text3',
                                type: 'text',
                                rect: ['921px', '14px', '99px', '32px', 'auto', 'auto'],
                                clip: 'rect(5.479248046875px 103.4658203125px 35.095703125px 12.32861328125px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Demo</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'play1',
                                type: 'image',
                                rect: ['943px', '0px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"play1.png",'0px','0px']
                            },
                            {
                                id: 'pause',
                                type: 'image',
                                rect: ['943px', '0px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"pause.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
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
                    duration: 5250,
                    autoPlay: false,
                    data: [
                        [
                            "eid368",
                            "left",
                            1000,
                            0,
                            "linear",
                            "${Hint_green}",
                            '-185px',
                            '-185px'
                        ],
                        [
                            "eid356",
                            "color",
                            1648,
                            352,
                            "linear",
                            "${Text}",
                            'rgba(0,0,0,0)',
                            'rgba(0,0,0,1.00)'
                        ],
                        [
                            "eid371",
                            "top",
                            1000,
                            1000,
                            "linear",
                            "${Hint_green}",
                            '-10px',
                            '-56px'
                        ],
                        [
                            "eid307",
                            "opacity",
                            1000,
                            1000,
                            "linear",
                            "${Hint_green}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid284",
                            "height",
                            1000,
                            905,
                            "linear",
                            "${Hint_green}",
                            '0px',
                            '228px'
                        ]
                    ]
                }
            },
            "video": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            preload: 'auto',
                            rect: ['-236px', '-11px', '1920', '1080', 'auto', 'auto'],
                            source: ['media/COLOR.webm'],
                            transform: [[], [], [], ['1.03', '1.03']],
                            id: 'demo2',
                            type: 'video',
                            clip: 'rect(117.68899536132813px 1645.4840087890625px 971.349365234375px 295.2447204589844px)',
                            tag: 'video'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            isStage: 'true',
                            rect: [undefined, undefined, '1920px', '1080px']
                        }
                    }
                },
                timeline: {
                    duration: 1500,
                    autoPlay: true,
                    data: [
                        [
                            "eid433",
                            "scaleY",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '0.55',
                            '1.03'
                        ],
                        [
                            "eid434",
                            "clip",
                            0,
                            250,
                            "linear",
                            "${demo2}",
                            [162.09091186523438,1644.2386474609375,922.5219116210938,284.22607421875],
                            [158.80662536621094,1645.145263671875,925.9461059570313,281.92706298828125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid448",
                            "clip",
                            250,
                            250,
                            "linear",
                            "${demo2}",
                            [158.80662536621094,1645.145263671875,925.9461059570313,281.92706298828125],
                            [155.5223388671875,1648.481201171875,929.3702392578125,285.2618408203125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid449",
                            "clip",
                            500,
                            192,
                            "linear",
                            "${demo2}",
                            [155.5223388671875,1648.481201171875,929.3702392578125,285.2618408203125],
                            [153,1645.310546875,932,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid435",
                            "clip",
                            692,
                            151,
                            "linear",
                            "${demo2}",
                            [153,1645.310546875,932,277.8624267578125],
                            [131.0282745361328,1644.8050537109375,946.640380859375,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid436",
                            "clip",
                            843,
                            657,
                            "linear",
                            "${demo2}",
                            [131.0282745361328,1644.8050537109375,946.640380859375,277.8624267578125],
                            [117.68899536132813,1645.4840087890625,971.349365234375,295.2447204589844],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid432",
                            "top",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '212px',
                            '-11px'
                        ],
                        [
                            "eid439",
                            "left",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '-551px',
                            '-236px'
                        ],
                        [
                            "eid437",
                            "height",
                            0,
                            0,
                            "linear",
                            "${demo2}",
                            '1080px',
                            '1080px'
                        ],
                        [
                            "eid438",
                            "scaleX",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '0.55',
                            '1.03'
                        ],
                        [
                            "eid440",
                            "width",
                            0,
                            0,
                            "linear",
                            "${demo2}",
                            '1930px',
                            '1930px'
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
                            rect: ['-62px', '0px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Exercice</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['137px', '-1px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">Exercice</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
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
                            [0,519.83837890625,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0160_Activite1_edgeActions.js");
})("EDGE-6689876");

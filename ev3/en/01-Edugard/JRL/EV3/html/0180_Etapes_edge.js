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
                            rect: ['86px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'A1_const2',
                            type: 'image',
                            rect: ['598px', '355px', '728px', '546px', 'auto', 'auto'],
                            clip: 'rect(123.30387115478516px 559.0736083984375px 411.71435546875px 177.9506378173828px)',
                            fill: ["rgba(0,0,0,0)",im+"A1_const2.jpg",'0px','0px'],
                            transform: [[],[],[],['0.81366','0.81366']]
                        },
                        {
                            id: 'A1_const3',
                            type: 'image',
                            rect: ['905px', '148px', '728px', '546px', 'auto', 'auto'],
                            clip: 'rect(121.54826354980469px 563.8013305664063px 411.71435546875px 178.48233032226563px)',
                            fill: ["rgba(0,0,0,0)",im+"A1_const3.jpg",'0px','0px'],
                            transform: [[],[],[],['0.81366','0.81366']]
                        },
                        {
                            id: 'A1_const11',
                            type: 'image',
                            rect: ['6px', '355px', '728px', '546px', 'auto', 'auto'],
                            clip: 'rect(122.21408081054688px 565.0471801757813px 411.714111328125px 171.5241241455078px)',
                            fill: ["rgba(0,0,0,0)",im+"A1_const11.jpg",'0px','0px'],
                            transform: [[],[],[],['0.81366','0.81366']]
                        },
                        {
                            id: 'A1_const1',
                            type: 'image',
                            rect: ['307px', '148px', '728px', '546px', 'auto', 'auto'],
                            clip: 'rect(122.857177734375px 559.6202392578125px 407.3601379394531px 178.13787841796875px)',
                            fill: ["rgba(0,0,0,0)",im+"A1_const1.jpg",'0px','0px'],
                            transform: [[],[],[],['0.81366','0.81366']]
                        },
                        {
                            id: '_1',
                            type: 'image',
                            rect: ['381px', '789px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"12.png",'0px','0px']
                        },
                        {
                            id: 'Text3',
                            type: 'group',
                            rect: ['161', '290', '1171', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'bloc',
                                type: 'image',
                                rect: ['55px', '104px', '580px', '361px', 'auto', 'auto'],
                                clip: 'rect(0px 580px 361px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"A1_sketch.png",'0px','0px']
                            },
                            {
                                id: 'text',
                                type: 'text',
                                rect: ['669px', '112px', '491px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">&nbsp;Oz will be a small robot of 45 centimeters wide and one meter long. It will be able to move independently in any type of crop​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titre2',
                                type: 'text',
                                rect: ['300px', '0px', '912px', '116px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: bold; font-style: normal; text-decoration: none; font-size: 51px; color: rgb(187, 117, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">Design:&nbsp;</span>it's time to innovate<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: bold; font-style: normal; text-decoration: none; font-size: 51px; color: rgb(187, 117, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\"></span></p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text7',
                            type: 'group',
                            rect: ['295', '290', '1119', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titre2Copy',
                                type: 'text',
                                rect: ['116px', '-2px', '972px', '112px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: bold; font-style: normal; text-decoration: none; font-size: 51px; color: rgb(187, 117, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">Prototype:&nbsp;</span>Make a non-definitive example</p><p style=\"margin: 0px; text-align: left;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: bold; font-style: normal; text-decoration: none; font-size: 51px; color: rgb(187, 117, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\"></span></p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'textee',
                                type: 'text',
                                rect: ['582px', '112px', '459px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">The prototype of Oz is well ready according to all that has been done before. Guided by a laser camera, equipped with a GPS, with four-wheel drive mode, Oz can work up to ten hours per day​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'hommes',
                                type: 'image',
                                rect: ['-80px', '91px', '629px', '378px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_Oz.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Text2',
                            type: 'group',
                            rect: ['188px', '290px', '1172', '477', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titre',
                                type: 'text',
                                rect: ['273px', '-4px', '1000px', '125px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 51px; color: rgb(187, 117, 0); font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Ideation :</span>Find an idea and establish the tasks that the robot will have to accomplish</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['625px', '127px', '602px', '306px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 32px;\"></span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; font-size: 32px;\"></span></p><p style=\"margin: 0px;\"><span style=\"font-size: 32px;\"></span>Oz is a robot farmer that helps to weed fields and hoe crops It improves productivity, make the work easier and preserves the environment<span style=\"font-size: 32px;\">​</span></p><p style=\"margin:0px\"><span style=\"font-size: 32px;\">​</span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'image_idee',
                                type: 'image',
                                rect: ['56px', '113px', '512px', '352px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_herbe.png",'0px','0px']
                            }]
                        },
                        {
                            id: '_2',
                            type: 'image',
                            rect: ['685px', '789px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: '_2Copy',
                            type: 'image',
                            rect: ['685px', '789px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: '_3',
                            type: 'image',
                            rect: ['960px', '789px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: '_4',
                            type: 'image',
                            rect: ['1227px', '789px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"42.png",'0px','0px']
                        },
                        {
                            id: 'b1',
                            type: 'image',
                            rect: ['381px', '789px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"b12.png",'0px','0px']
                        },
                        {
                            id: 'b2',
                            type: 'image',
                            rect: ['685px', '789px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b22.png",'0px','0px']
                        },
                        {
                            id: '_3Copy',
                            type: 'image',
                            rect: ['960px', '789px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: 'b3',
                            type: 'image',
                            rect: ['960px', '789px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b32.png",'0px','0px']
                        },
                        {
                            id: '_4Copy',
                            type: 'image',
                            rect: ['1227px', '789px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"42.png",'0px','0px']
                        },
                        {
                            id: 'b4',
                            type: 'image',
                            rect: ['1227px', '789px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b42.png",'0px','0px']
                        },
                        {
                            id: 'Text',
                            type: 'group',
                            rect: ['312px', '119px', '803', '169', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ROBOT08',
                                type: 'image',
                                rect: ['0px', '0px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['136px', '1px', '740px', '141px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'TextIN',
                                type: 'text',
                                rect: ['193px', '28px', '643px', '141px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 36px;\">We give you the 4 steps necessary to create your robot:</span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Rect_liaison_gris',
                            type: 'rect',
                            rect: ['489px', '837px', '200px', '5px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_vert',
                            type: 'rect',
                            rect: ['486px', '837px', '200px', '0.5%', 'auto', 'auto'],
                            fill: ["rgba(255,170,74,1.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_grisCopy2',
                            type: 'rect',
                            rect: ['792px', '837px', '175px', '5px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_vertCopy2',
                            type: 'rect',
                            rect: ['789px', '837px', '170px', '0.5%', 'auto', 'auto'],
                            fill: ["rgba(255,170,74,1.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_grisCopy3',
                            type: 'rect',
                            rect: ['1065px', '837px', '165px', '5px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_vertCopy3',
                            type: 'rect',
                            rect: ['1064px', '837px', '165px', '0.5%', 'auto', 'auto'],
                            fill: ["rgba(255,170,74,1.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Text9',
                            type: 'group',
                            rect: ['295px', '290px', '1128', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titre2Copy2',
                                type: 'text',
                                rect: ['80px', '0px', '1138px', '144px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: bold; font-style: normal; text-decoration: none; font-size: 51px; color: rgb(187, 117, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">Programming and Tests:&nbsp;</span>Bring the robot to life!!</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'tesxteee',
                                type: 'text',
                                rect: ['619px', '113px', '491px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"></p><p style=\"margin: 0px;\"></p><p style=\"margin: 0px;\">All the tasks to be performed by the Oz robot are programmed and it is being tested. The goal is to identify errors and make the necessary changes after each trial.​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Oz_test',
                                type: 'video',
                                tag: 'video',
                                rect: ['79px', '-2px', '960', '540', 'auto', 'auto'],
                                clip: 'rect(85px 785px 449px 134.66259765625px)',
                                source: [vid+"Oztest.webm"],
                                preload: 'auto'
                            },
                            {
                                id: 'play_boutton',
                                type: 'image',
                                rect: ['-79px', '98px', '653px', '340px', 'auto', 'auto'],
                                clip: 'rect(5.5556640625px 496.05517578125px 340px 152.77734375px)',
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"play.png",'0px','0px'],
                                transform: [[],[],[],['0.37708','0.37708']]
                            },
                            {
                                id: 'cache_cadre',
                                type: 'image',
                                rect: ['-208px', '588px', '1450px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'full_screen',
                            type: 'image',
                            rect: ['792px', '680px', '46px', '47px', 'auto', 'auto'],
                            clip: 'rect(0px 60px 60px 0px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                        },
                        {
                            id: 'pointer',
                            type: 'image',
                            rect: ['460px', '868px', '34px', '52px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['577px', '36px', '390', '57', 'auto', 'auto']
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
                            "eid331",
                            "top",
                            5000,
                            500,
                            "linear",
                            "${Oz_test}",
                            '-2px',
                            '-53px'
                        ],
                        [
                            "eid182",
                            "width",
                            750,
                            500,
                            "linear",
                            "${Rect_liaison_vert}",
                            '0px',
                            '200px'
                        ],
                        [
                            "eid330",
                            "height",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '52px',
                            '57px'
                        ],
                        [
                            "eid328",
                            "top",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '680px',
                            '815px'
                        ],
                        [
                            "eid329",
                            "width",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '51px',
                            '56px'
                        ],
                        [
                            "eid332",
                            "scaleY",
                            5000,
                            500,
                            "linear",
                            "${Oz_test}",
                            '1',
                            '2.00798'
                        ],
                        [
                            "eid191",
                            "width",
                            1500,
                            500,
                            "linear",
                            "${Rect_liaison_vertCopy2}",
                            '0px',
                            '170px'
                        ],
                        [
                            "eid335",
                            "clip",
                            5000,
                            148,
                            "linear",
                            "${Oz_test}",
                            [85,785,449,134.66259765625],
                            [69.21028900146484,827.0865478515625,481.42535400390625,130.33428955078125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid336",
                            "clip",
                            5148,
                            164,
                            "linear",
                            "${Oz_test}",
                            [69.21028900146484,827.0865478515625,481.42535400390625,130.33428955078125],
                            [68.66205596923828,826.13232421875,480.3761901855469,133.5812530517578],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid337",
                            "clip",
                            5312,
                            188,
                            "linear",
                            "${Oz_test}",
                            [68.66205596923828,826.13232421875,480.3761901855469,133.5812530517578],
                            [62.92515563964844,784.8436889648438,495.62799072265625,134.40005493164062],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid333",
                            "scaleX",
                            5000,
                            500,
                            "linear",
                            "${Oz_test}",
                            '1',
                            '2.13459'
                        ],
                        [
                            "eid327",
                            "left",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '792px',
                            '1432px'
                        ],
                        [
                            "eid192",
                            "width",
                            2250,
                            500,
                            "linear",
                            "${Rect_liaison_vertCopy3}",
                            '0px',
                            '165px'
                        ],
                        [
                            "eid334",
                            "left",
                            5000,
                            500,
                            "linear",
                            "${Oz_test}",
                            '-228px',
                            '79px'
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
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(0, 61, 65); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">Creation steps</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['27px', '-11px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; text-transform: uppercase; font-weight: 500;\">Creation steps</span></p>',
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0180_Etapes_edgeActions.js");
})("EDGE-6689876");

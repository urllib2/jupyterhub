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
                            id: 'Group1',
                            type: 'group',
                            rect: ['380px', '645px', '600', '39', 'auto', 'auto'],
                            c: [
                            {
                                id: 'BackGround3',
                                type: 'rect',
                                rect: ['51px', '-14px', '632px', '66px', 'auto', 'auto'],
                                fill: ["rgba(208,208,208,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'BarKhowledge',
                                type: 'rect',
                                rect: ['62px', '0px', '600px', '39px', 'auto', 'auto'],
                                fill: ["rgba(51,51,51,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'BarKhowledgeProgress',
                                type: 'rect',
                                rect: ['62px', '0px', '20px', '39px', 'auto', 'auto'],
                                fill: ["rgba(35,201,87,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'PourcentageConn',
                                type: 'text',
                                rect: ['701px', '-13px', '217px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px;\">​</span><span style=\"font-weight: 600; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px;\">​​1</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(35,167,169,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Conaissance',
                                type: 'text',
                                rect: ['-188px', '-5px', 'auto', 'auto', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<font color=\"#6309c9\" face=\"Trebuchet MS, Arial, Helvetica, sans-serif\"><span style=\"font-size: 36px;\"><b>Khowledge</b></span></font></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "nowrap"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Group2',
                            type: 'group',
                            rect: ['382px', '546px', '600', '39', 'auto', 'auto'],
                            c: [
                            {
                                id: 'BackGround2',
                                type: 'rect',
                                rect: ['51px', '-14px', '632px', '66px', 'auto', 'auto'],
                                fill: ["rgba(208,208,208,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'BarBuilding',
                                type: 'rect',
                                rect: ['61px', '0px', '600px', '39px', 'auto', 'auto'],
                                fill: ["rgba(51,51,51,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'BarBuildingProg',
                                type: 'rect',
                                rect: ['61px', '0px', '20px', '39px', 'auto', 'auto'],
                                fill: ["rgba(35,201,87,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'Mecanique',
                                type: 'text',
                                rect: ['-181px', '-9px', 'auto', 'auto', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px; color: rgb(99, 9, 201); font-weight: 700;\">Building</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "nowrap"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'PourcentageMec',
                                type: 'text',
                                rect: ['701px', '-9px', '217px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px; font-weight: 600;\">1​</span><span style=\"font-weight: 600;\">​</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 600;\">​</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px; font-weight: 600;\">​​</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px; font-weight: 600; color: rgb(255, 0, 0);\">​</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(35,167,169,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Group3',
                            type: 'group',
                            rect: ['394px', '750px', '600', '39', 'auto', 'auto'],
                            c: [
                            {
                                id: 'BackGround',
                                type: 'rect',
                                rect: ['39px', '-13px', '632px', '66px', 'auto', 'auto'],
                                fill: ["rgba(208,208,208,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'BarCoding',
                                type: 'rect',
                                rect: ['52px', '0px', '600px', '39px', 'auto', 'auto'],
                                fill: ["rgba(51,51,51,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'BarCodingProgress',
                                type: 'rect',
                                rect: ['52px', '0px', '20px', '39px', 'auto', 'auto'],
                                fill: ["rgba(35,201,87,1.00)"],
                                stroke: [0,"rgb(0, 0, 0)","none"]
                            },
                            {
                                id: 'Programation',
                                type: 'text',
                                rect: ['-199px', '-2px', 'auto', 'auto', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 36px; color: rgb(99, 9, 201); font-weight: 700; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Coding</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "nowrap"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'PourcentageProg',
                                type: 'text',
                                rect: ['689px', '-13px', '217px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px; font-weight: 600;\">1​</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(35,167,169,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'bouton',
                            type: 'group',
                            rect: ['333px', '791px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
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
                                rect: ['925px', '16px', '153px', '47px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​Results</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Resultat',
                            type: 'group',
                            rect: ['456px', '412px', '1025', '111', 'auto', 'auto'],
                            clip: 'rect(0px 976.2822265625px 110px 0px)',
                            c: [
                            {
                                id: 'Message',
                                type: 'text',
                                rect: ['55px', '28px', '1025px', '83px', 'auto', 'auto'],
                                clip: 'rect(0px 872.918212890625px 83px 0px)',
                                text: "<p style=\"margin: 0px;\">You Got &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;/ &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; stars</p>",
                                font: ['MagistralC', [49, "px"], "rgba(0,0,0,1)", "600", "none", "", "break-word", "normal"],
                                transform: [[],[],[],['1.10575']]
                            },
                            {
                                id: 'Total',
                                type: 'text',
                                rect: ['448px', '8px', '153px', '75px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-family: MagistralC; font-weight: 800; font-style: normal; text-decoration: none; font-size: 87px; color: rgb(193, 128, 2); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">0</p>",
                                align: "left",
                                font: ['MagistralC', [91, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Nombre',
                                type: 'text',
                                rect: ['281px', '8px', '207px', '75px', 'auto', 'auto'],
                                clip: 'rect(0px 166.76953125px 95.512939453125px 0px)',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-weight: 800; font-size: 87px; color: rgb(193, 128, 2);\">0 &nbsp;</span></p>",
                                align: "left",
                                font: ['MagistralC', [91, "px"], "rgba(193,128,2,1.00)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Remarque',
                            type: 'text',
                            rect: ['834px', '141px', '567px', '82px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            align: "center",
                            font: ['MagistralC', [55, "px"], "rgba(0,0,0,1)", "600", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'Bar',
                            type: 'image',
                            rect: ['381px', '219px', '912px', '198px', 'auto', 'auto'],
                            opacity: '0.5',
                            fill: ["rgba(0,0,0,0)",im+"Bar_Op.png",'0px','0px']
                        },
                        {
                            id: 'Colors',
                            type: 'group',
                            rect: ['380px', '219px', '915', '198', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Red',
                                type: 'image',
                                rect: ['0px', '0px', '913px', '198px', 'auto', 'auto'],
                                clip: 'rect(7px 891px 160px 743px)',
                                fill: ["rgba(0,0,0,0)",im+"Red.png",'0px','0px']
                            },
                            {
                                id: 'Orange',
                                type: 'image',
                                rect: ['2px', '0px', '913px', '198px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Orange.png",'0px','0px']
                            },
                            {
                                id: 'Yellow',
                                type: 'image',
                                rect: ['13px', '0px', '886px', '198px', 'auto', 'auto'],
                                clip: 'rect(10px 585px 160px 446px)',
                                fill: ["rgba(0,0,0,0)",im+"Yellow.png",'0px','0px'],
                                transform: [[],[],[],['1.03047']]
                            },
                            {
                                id: 'Yellow1',
                                type: 'image',
                                rect: ['2px', '0px', '913px', '198px', 'auto', 'auto'],
                                clip: 'rect(8px 459px 157px 318px)',
                                fill: ["rgba(0,0,0,0)",im+"Yellow1.png",'0px','0px']
                            },
                            {
                                id: 'LightGreen',
                                type: 'image',
                                rect: ['2px', '0px', '913px', '198px', 'auto', 'auto'],
                                clip: 'rect(8px 319px 161px 178px)',
                                fill: ["rgba(0,0,0,0)",im+"LightGreen.png",'0px','0px']
                            },
                            {
                                id: 'Green',
                                type: 'image',
                                rect: ['0px', '0px', '913px', '198px', 'auto', 'auto'],
                                clip: 'rect(5px 179px 163px 27px)',
                                fill: ["rgba(0,0,0,0)",im+"Green.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Aiguille',
                            type: 'image',
                            rect: ['1170px', '360px', '48px', '57px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"aiguille.png",'0px','0px']
                        },
                        {
                            id: 'Entete',
                            type: 'text',
                            rect: ['305px', '119px', '824px', '100px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 91px;\">Your Score is</span></p>",
                            font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['604px', '33px', '390', '57', 'auto', 'auto']
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
                            "eid219",
                            "top",
                            3000,
                            2250,
                            "linear",
                            "${Aiguille}",
                            '415px',
                            '360px'
                        ],
                        [
                            "eid218",
                            "left",
                            3000,
                            2250,
                            "linear",
                            "${Aiguille}",
                            '1174px',
                            '1170px'
                        ]
                    ]
                }
            },
            "TITRE": {
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
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['67px', '-4px', '655px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Results</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['67px', '-4px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 636px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Results</span></p>'
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
                            [0,636,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0310_Evaluation_edgeActions.js");
})("EDGE-6689876");

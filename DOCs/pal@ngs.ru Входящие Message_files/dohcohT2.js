window.ngs_avc = /(;\s*|^)ngs_avc=(\d+)/.exec(document.cookie + '; ngs_avc=' + (Math.random()*12|0))[2]*1;

window.alflv = function(){
    try {
        try {
            var axo = new ActiveXObject('ShockwaveFlash.ShockwaveFlash.6');
            try { axo.AllowScriptAccess = 'always'; }
            catch(e) { return 6; }
        } catch(e) {}
        return new ActiveXObject('ShockwaveFlash.ShockwaveFlash').GetVariable('$version').match(/\d+/)[0];
    } catch(e) {
        try {
            if(navigator.mimeTypes["application/x-shockwave-flash"].enabledPlugin){
                return (navigator.plugins["Shockwave Flash 2.0"] || navigator.plugins["Shockwave Flash"]).description.match(/\d+/)[0];
            }
        } catch(e) {}
    }
    return 0;
}();

function albex(pid, w, h, l, t) {
    if (typeof(pid) != 'undefined') {
        var o = document.getElementById('ap' + pid);

        if (o) {
            if (typeof(w) != 'undefined') {
                o.style.position = 'absolute';
                o.style.zIndex = '999';

                if (typeof(l) != 'undefined') {
                    o.style.left = '-' + l + 'px';
                    o.style.top = '-' + t + 'px';
                }

                if (w) {
                    o.style.width = w + 'px';
                }

                if (h) {
                    o.style.height = h + 'px';
                }

                if (typeof o.SetVariable != 'function') {
                    o.SetVariable('expand', '1');
                }

            } else {
                o.style.position = '';
                o.style.zIndex = '';
                o.style.width = '100%';
                o.style.height = o.parentNode.style.height || o.parentNode.parentNode.style.height;
                o.style.left = '0';
                o.style.top = '';

                if (typeof o.SetVariable != 'function') {
                    o.SetVariable('expand', '');
                }
            }
        }
    }
};

function fr3(v, t, into, style) {

    var placeId = /id=\"ap([0-9]+)|pid=([0-9]+)/.exec(t);
    if (placeId) {
        placeId = placeId[1] || placeId[2];
        if (localStorage.getItem('closedPlace'+placeId) > new Date().getTime() - 14400000) {//Баннер скрыли меньше 4х часов назад
            return;
        } else {
            localStorage.removeItem('closedPlace'+placeId);
        }
    }

    if (document.titlebk === undefined) document.titlebk = document.title;

    if (v > 7 && window.alflv < 8) {
        t = t.replace(/<div[^>]*><div[^>]*><object[^>]*>(<param[^>]*>)*([\S\s]+?)<\/object><\/div><\/div>/, '$2');
    }

    if (window.chrome) {
        t = t.replace(/(<object) (style="width:)(?! 1px) ([^;]+)(; height:) ([^;]+)/, '$1 data-width="$3" data-height="$5" $2 1024px; visibility: hidden$4 768px');

        var oid = /<object[^>]*\sid=['"]([^'"]+)/i.exec(t);
        if (oid) {
            oid = oid[1];
            var wait = 100;
            window.setTimeout(function swX() {
                wait *= 1.1;
                var o = document.getElementById(oid);
                if (o && o.tagName.toLowerCase() === 'object' && typeof o.PercentLoaded != 'undefined' && o.PercentLoaded() === 100) {
                    if (o.getAttribute('data-width')) {
                        o.style.width = o.getAttribute('data-width');
                        o.style.height = o.getAttribute('data-height');
                        o.removeAttribute('data-width');
                        o.removeAttribute('data-height');
                        o.style.visibility = 'visible';
                    }
                } else {
                    window.setTimeout(swX, wait);
                }
            }, wait);
        }
    }

    if (into === undefined) {
        document.write(t);
    } else {
        intoBlk = document.getElementById('ngs-al-' + into);
        if (intoBlk) {
            if (t != '') {
                intoBlk.innerHTML = t;
                if (style) {
                    intoBlk.style.cssText = style + intoBlk.style.cssText;
                }
                intoBlk.style.display = '';
            } else {
                intoBlk.style.display = 'none';
            }
        } else {
            window.ngs_async_banners = window.ngs_async_banners || {};
            window.ngs_async_banners[into] = t;
        }
    }
};

function fr3ck() {
    if (window.ngs_async_banners) {
        for (var into in window.ngs_async_banners) {
            var t = window.ngs_async_banners[into];
            intoBlk = document.getElementById('ngs-al-' + into);
            if (intoBlk) {
                if (t != '') {
                    intoBlk.innerHTML = t;
                    intoBlk.style.display = '';
                } else {
                    intoBlk.style.display = 'none';
                }
            }
        }
    }

    window.setTimeout(
        function(){
            if (document.titlebk !== undefined) document.title = document.titlebk;

            var ab = 0;
            var objs = document.getElementsByTagName('object');
            var at = objs.length;

            if (objs.length) {
                var o, i = objs.length;
                while (i) {
                    o = objs.item(--i);
                    try {
                        if (typeof o.PercentLoaded === 'undefined' || o.PercentLoaded() == 0) {
                            ab++;
                        }
                    } catch (xc) {
                        ab++;
                    }

                    o = o.parentNode;
                    if (o.style.position == 'fixed' && o.style.width == '1px') {
                        o.parentNode.removeChild(o);
                    }
                }
            } else {
                objs = document.getElementsByTagName('script');
                var o, i = 0;
                while (o = objs.item(i++)) {
                    if (o.innerHTML && o.innerHTML.indexOf('<object ') > -1) {
                        at++;
                        ab++;
                    }
                }
            }

            objs = document.getElementsByTagName('div');
            if (objs.length) {
                var o, i = 0;
                while (o = objs.item(i++)) {
                    if (o.style.backgroundImage && o.style.backgroundImage.indexOf('flash') > -1) {
                        at++;
                        ab++;
                    }
                }
            }
            if (at) {
                var doh = /(;\s*|^)doh=(\d+)/.exec(document.cookie + '; doh=10')[2]*1;
                document.cookie = 'doh='+(ab/at > 0.9? doh + (doh < 10) : 0)+'; expires='+(new Date((new Date()).getTime() + 600000000)).toGMTString()+'; path=/' + (location.hostname? '; domain=.' + /([-a-z0-9]+)\.(ru|ua|by|com|su|gs|xn--p1ai)((\.[-a-z0-9]+)?\.[dt]\d?)?$/i.exec(location.hostname)[0] : '');
            }

            if (document.getElementsByClassName) {
                var o = document.getElementsByClassName('advplace-num');
                if (o.length) {
                    o[0].scrollIntoView(false);
                }
            }

            var adp_params = /(?:(?:_adpreview|_apx|_aph)=[^&]+&?){3,}/.exec(window.location.search);
            if (adp_params) {
                var script = document.createElement('script');
                script.src = '//reklama.ngs.ru/placen.js?' + adp_params[0];
                document.body.appendChild(script);
            }

            if (document.cookie.indexOf('_adpreview=') != -1) {
                var o = document.getElementById('ap' + /(;\s*|^)_adpreview=(\d+)/.exec(document.cookie)[2]);
                if (o) {
                    o.scrollIntoView(false);
                } else {
                    document.body.parentNode.removeChild(document.body);
                }
            }
        },
        500
    );

    window.advSizeUp = window.setInterval(
        function () {
            var f = false;
            var objs = document.getElementsByTagName('div');
            if (objs.length) {
                var o, s, i = 0;
                while (o = objs.item(i++)) {
                    if (o.className.match(/^advplace-preview/) && (s = o.getElementsByTagName('span')) && s[0].className == 'size') {
                        s[0].innerHTML = ' / ' + o.scrollWidth + '*' + o.scrollHeight;
                        f = true;
                    }
                }
            }
            if (!f) {
                window.clearInterval(window.advSizeUp);
            }
        },
        500
    );

    if (window.adpolls) {
        for (var pn in window.adpolls) {
            var mya = RegExp('(;\\s*|^)adpoll'+pn+'=(\\d+)/(\\d+)').exec(document.cookie);
            if (mya) {
                var myats = mya[3];
                mya = mya[2];
                if (myats < window.adpolls[pn].ts) {
                    adpollSubmit(pn, -1);
                } else {
                    adpollSubmit(pn, mya);
                }
            } else {
                document.getElementById('adpoll'+pn+'-results').style.display='none';
                document.getElementById('adpoll'+pn+'-poll').style.display='block';
                document.getElementById('adpoll'+pn+'-btm-imgb-div').style.display='none';
                document.getElementById('adpoll'+pn+'-btm-imga-div').style.display='block';
            }
        }
    }

    document.cookie = 'ngs_avc='+((ngs_avc+1)%12)+'; expires='+(new Date((new Date()).getTime() + 600000000)).toGMTString()+'; path=/';
};

function adpollSubmit(pn, show) {
    var a;
    var i;
    if (show === undefined && pn) {
        a = document.getElementById('adpoll'+pn+'-poll').getElementsByTagName('input');
        i = a.length;
        while (i--) {
            if (a[i].checked) {
                a = a[i].value;
                break;
            }
        }
        if (typeof a == 'object') return;
        document.cookie = 'adpoll'+pn+'='+a+'/'+window.adpolls[pn].ts+'; expires='+(new Date((new Date()).getTime() + 2600000000)).toGMTString()+'; path=/' + (location.hostname? '; domain=.' + /[-a-z0-9]+\.([a-z]+|xn--[-a-z0-9]+)$/i.exec(location.hostname)[0] : '');

        var ti = document.createElement("img");
        ti.src = document.getElementById('adpoll'+pn+'-btm-imgb-div').getElementsByTagName('a')[0].href + '?pa=' + a;
    } else {
        a = show;
    }

    if (a >= 0) {
        window.adpolls[pn].c[a]++;
    }

    var sum = 0;

    i = window.adpolls[pn].c.length;
    while (i--) {
        sum += window.adpolls[pn].c[i];
    }

    a = document.getElementById('adpoll'+pn+'-results').getElementsByTagName('tr');
    i = a.length;
    while (i--) {
        a[i].getElementsByTagName('td')[0].innerHTML = Math.round(sum ? 100 * window.adpolls[pn].c[i] / sum : 0)+' %';
        a[i].getElementsByTagName('td')[1].innerHTML = window.adpolls[pn].c[i];
    }

    document.getElementById('adpoll'+pn+'-results').style.display='block';
    document.getElementById('adpoll'+pn+'-poll').style.display='none';
    document.getElementById('adpoll'+pn+'-btm-imgb-div').style.display='block';
    document.getElementById('adpoll'+pn+'-btm-imga-div').style.display='none';
};

function ngs_adplace(pid, params, into) {
    if (/string|number/.test(typeof params)) {
        into = params;
        params = {}
    }

    params = params || {};

    var qs = 'pid=' + pid;

    qs += '&sn=' + (params['sn'] || location.hostname);

    qs += '&hh=' + location.hostname;

    qs += '&ru=' + (params['ru'] || location.pathname).slice(1);

    qs += '&ts=' + (new Date().getTime() / 1000 | 0);

    qs += '&avc=' + ngs_avc;

    qs += '&doh=' + /(;\s*|^)doh=(\d+)/.exec(document.cookie + '; doh=0')[2]*1;

    var adp_params = /(?:(?:_adpreview|_apx|_aph)=[^&]+&?)+/.exec(window.location.search);
    if(adp_params) {
        qs = adp_params[0] + '&' + qs;
    }

    if (into === undefined) {
        document.write('<script src="//reklama.ngs.ru/ap-js/?' + qs + '"></script>');
    } else {
        qs += '&asb=' + into;

        var s = document.createElement("script");
        s.src = '//reklama.ngs.ru/ap-js/?' + qs;
        s.async = true;
        document.body.appendChild(s);
    }
}

if (window.addEventListener) {
    window.addEventListener('load', fr3ck, false);
} else if (window.attachEvent) {
    window.attachEvent('onload', fr3ck);
}

window.alLoadUnload = function me () {
    if (me.locked) {
        if (!me.waiting) {
            me.waiting = true;
            setTimeout(function () {me.locked = false; me();}, 200);
        }
        return;
    }
    me.locked = true;
    me.waiting = false;
    var objs = document.getElementsByTagName('iframe');
    if (objs.length) {
        var o, i = 0;
        while (o = objs.item(i++)) {
            if (!o.getAttribute('data-src-al')) {
                continue;
            }

            var rect = o.getBoundingClientRect();

            var vpHeight = window.innerHeight || document.documentElement.clientHeight;
            var vpWidth = window.innerWidth || document.documentElement.clientWidth;

            if (
                rect.bottom >= (-vpHeight / 2) &&
                rect.right >= (-vpWidth / 2) &&
                rect.top <= (vpHeight * 1.5) &&
                rect.left <= (vpWidth * 1.5)
            ) {
                if (!o.alActive) {
                    o.contentWindow.location.replace(o.getAttribute('data-src-al'));
                    o.alActive = true;
                } else {
                    //o.contentWindow.postMessage('start', '*');
                }
            } else {
                if (rect.width * rect.height > 100000 && rect.width > rect.height && rect.width < rect.height * 3) {
                    continue;
                }

                if (o.alActive) {
                    o.contentWindow.location.replace('about:blank');
                    o.alActive = false;
                }
                //o.contentWindow.postMessage('pause', '*');
            }
        }
    }
};

(function (f) {
    if (window.addEventListener) {
        addEventListener('DOMContentLoaded', f, false);
        addEventListener('load', f, false);
        addEventListener('scroll', f, false);
        addEventListener('resize', f, false);
        document.addEventListener('visibilitychange', f, false);
    } else if (window.attachEvent)  {
        attachEvent('onDOMContentLoaded', f);
        attachEvent('onload', f);
        attachEvent('onscroll', f);
        attachEvent('onresize', f);
        document.attachEvent('onvisibilitychange', f);
    }
})(window.alLoadUnload);

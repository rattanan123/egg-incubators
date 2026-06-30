const CACHE = 'egg-v3';
const ASSETS = ['/egg-incubator/', '/egg-incubator/index.html', '/egg-incubator/dashboard.html', '/egg-incubator/login.html'];

self.addEventListener('install', e => {
  e.waitUntil(caches.open(CACHE).then(c => c.addAll(ASSETS)));
  self.skipWaiting();
});

self.addEventListener('activate', e => {
  e.waitUntil(caches.keys().then(keys =>
    Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k)))
  ));
  self.clients.claim();
});

self.addEventListener('fetch', e => {
  if (e.request.url.includes('firebasedatabase') || e.request.url.includes('googleapis')) return;
  e.respondWith(
    fetch(e.request).catch(() => caches.match(e.request))
  );
});

self.addEventListener('push', e => {
  const data = e.data?.json() || {};
  e.waitUntil(self.registration.showNotification(data.title || 'ตู้ฟักไข่', {
    body: data.body || '',
    icon: '/egg-incubator/icon-192.png',
    badge: '/egg-incubator/icon-192.png'
  }));
});
